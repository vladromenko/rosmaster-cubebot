"""Experiment 5: interactive assistant: "find the cube, bring it to me and give it to my hand".

The command is parsed by rules (Russian/English; no network dependency). Every physical step runs in the verified
modules: the cube/object pipeline of experiment 4, the person tracker and follow controller of experiment 1, the arm IK.
A handover is only reported when there is physical evidence (pull on the servos + hand at the gripper, then the object is
seen in the hand); reaching the target coordinate alone never counts. If the hand is out of the arm's reach the result is
"handover unreachable" (an unfinished feature), never a success.
"""
import math
import re
import time

import numpy as np

from ..geometry import wrap
from .common import Experiment, base_limits, draw_people, hist_distance, put_text
from .follow import FollowRunner
from .motion import translate, turn_by
from .retrieve import Retriever, resolve_target

FIND_WORDS = ("найд", "найти", "принес", "достан", "подай", "подать", "отдай", "доставь", "find", "bring", "fetch", "get")
PERSON_WORDS = ("мне", "ко мне", "принес", "отдай", "доставь", "to me", "bring", "give")
HAND_WORDS = ("в руку", "в руки", "руку", "hand", "into my")


def parse_command(text, ecfg):
    """Rule-based command parser. Returns dict(ok, intent, object, to_person, handover, text) or dict(ok=False, error)."""
    t = " " + text.lower().strip() + " "
    t = re.sub(r"[,.!?;:]", " ", t)
    label = resolve_target(t, ecfg)
    if any(w in t for w in ("стоп", "остановись", " stop ")):
        return dict(ok=True, intent="stop", text=text)
    if label and any(w in t for w in FIND_WORDS):
        to_person = any(w in t for w in PERSON_WORDS)
        hand = any(w in t for w in HAND_WORDS)
        return dict(ok=True, intent="fetch", object=label, to_person=to_person or hand,
                    handover="hand" if (hand or to_person) else None, text=text)
    if any(w in t for w in ("ко мне", "подъедь", "иди сюда", "come here")):
        return dict(ok=True, intent="come", text=text)
    return dict(ok=False, error="не понял команду: назовите предмет и действие, например «найди кубик, принеси его мне и отдай в руку»",
                text=text)


class AssistantExperiment(Experiment):
    key = "e5_assistant"
    title = "5. Робот-помощник"
    STEPS = ("command", "requester", "found_object", "grasped", "found_person", "approached", "hand_seen",
             "reachable", "pulled", "handed_over")

    def __init__(self, ctx, nav_getter):
        super().__init__(ctx)
        self.nav_getter = nav_getter
        self.ac = ctx.cfg["exp"]["assistant"]
        self.steps = {}
        self.step = ""
        self.evidence = {}
        self.runner = None
        self.requester = None
        self.retriever = None
        self.plan = None

    # ---- lifecycle
    def setup(self):
        self.ac = self.ecfg["assistant"]
        self.steps, self.evidence, self.requester, self.retriever = {}, {}, None, None
        self.metrics = {}
        self.step = ""
        self.allow_runtime(float(self.params.get("runtime_s", 1200)))
        self.ctx.perception.tracker.reset()
        self.plan = parse_command(self.params.get("text", ""), self.ecfg)
        self.state = "running"

    def teardown(self):
        self.ctx.perception.set_want(people=False, hands=False, pose=False)
        self.ctx.perception.crop = None
        self.ecfg["detector"]["people_every"] = 1
        self.io.stop_base()

    def mark(self, name, ok=True, **evidence):
        self.steps[name] = bool(ok)
        if evidence:
            self.evidence[name] = evidence
        self.metrics = dict(step=self.step, steps=dict(self.steps), evidence=self.evidence)

    def fail(self, reason, **kw):
        self.result = "не выполнено: %s" % reason
        self.state = "failed"
        self.say(self.result)
        self.finish(False, reason, **kw)

    def finish(self, ok, reason=None, **extra):
        rec = dict(kind="scenario", ok=bool(ok), command=self.params.get("text"), plan=self.plan, steps=self.steps,
                   evidence=self.evidence, reason=reason, note=self.params.get("note", ""), video=self._recorder.path,
                   seconds=round(time.monotonic() - self.started_at, 1), **extra)
        self.ctx.store.add(self.key, rec)

    # ---- main scenario
    def run(self):
        self.step = "command"
        plan = self.plan
        self.mark("command", plan["ok"], plan=plan)
        if not plan["ok"]:
            return self.fail(plan["error"])
        self.say("команда: %s" % {k: v for k, v in plan.items() if k != "text"})
        if plan["intent"] == "stop":
            return self.finish(True, "stop")
        if plan["intent"] == "come":
            return self.come_to_person()
        if plan["intent"] == "fetch" and not plan["to_person"]:
            return self.fetch_only()
        return self.fetch_and_hand_over()

    def come_to_person(self):
        self.ecfg["detector"]["people_every"] = 1
        track = self.register_requester()
        if track is None:
            return self.fail("не вижу человека перед роботом")
        ok = self.approach(self.requester)
        self.mark("approached", ok)
        self.result = "подъехал к человеку" if ok else "не удалось подъехать"
        self.state = "done" if ok else "failed"
        self.finish(ok, None if ok else "approach failed")

    def fetch_only(self):
        nav = self.nav_getter(create=False)
        r = Retriever(self.io, self.cfg, self.say, plan_label(self.plan), dict(kind="start"), self.ctx.perception,
                      nav if nav is not None and nav.running else None)
        self.retriever = r
        ok = r.run()
        self.mark("found_object", r.stages["detected"])
        self.mark("grasped", r.stages["grasped"])
        self.result = r.result
        self.state = "done" if ok else "failed"
        self.finish(ok, None if ok else r.fail_reason, stages=r.stages)

    def fetch_and_hand_over(self):
        io, ctx = self.io, self.ctx
        # 1. remember who asked: colour model + direction
        self.step = "requester"
        self.ecfg["detector"]["people_every"] = 1
        if self.register_requester() is None:
            return self.fail("не вижу человека, который дал команду: встаньте перед роботом на 1-3 м")
        # 2. find and grasp the object (experiment 4 pipeline, object stays in the gripper)
        self.step = "fetch"
        ctx.perception.set_want(people=False)
        nav = self.nav_getter(create=False)
        r = Retriever(io, self.cfg, self.say, plan_label(self.plan), dict(kind="hold"), ctx.perception,
                      nav if nav is not None and nav.running else None)
        self.retriever = r
        ok = r.run()
        self.mark("found_object", r.stages["detected"])
        self.mark("grasped", r.stages["grasped"])
        if not ok:
            reason = r.fail_reason or "fetch failed"
            return self.fail("предмет: %s" % reason, stages=r.stages)
        # 3. find the person again and drive up to them
        approached = False
        for attempt in range(1, 3):
            self.step = "find_person (try %d)" % attempt
            track = self.find_requester()
            self.mark("found_person", track is not None)
            if track is None:
                continue
            self.step = "approach_person"
            approached = self.approach(self.requester, track)
            self.mark("approached", approached)
            if approached:
                break
            self.say("человек сместился или пропал при подъезде: ищу снова")
        if not approached:
            return self.fail("человек не найден или недоступен (предмет остался в захвате)")
        # 4. outstretched hand
        self.step = "hand"
        hand = self.wait_hand()
        self.mark("hand_seen", hand is not None, xyz=None if hand is None else np.round(hand, 3).tolist())
        if hand is None:
            return self.fail("протянутая рука не обнаружена (предмет остался в захвате)")
        # 5. reachability with the arm (IK)
        self.step = "reach"
        for _ in range(2):
            ok_reach, servo, target, why = self.can_present(hand)
            if ok_reach:
                break
            if "x=" in why:                       # too far/near only: shift the base once
                dx = float(np.clip(hand[0] - 0.22, -0.25, 0.25))
                self.say("рука не в рабочей зоне по x: сдвигаюсь на %.2f м" % dx)
                translate(io, dx, 0.0, speed=0.10)
                hand = self.wait_hand() if True else hand
                if hand is None:
                    return self.fail("рука потеряна после сдвига")
                continue
            break
        self.mark("reachable", ok_reach, target=None if target is None else np.round(target, 3).tolist(), why=why)
        if not ok_reach:
            return self.fail("передача в руку недостижима для манипулятора: %s. Функция не завершена; "
                             "предмет остался в захвате" % why, unfinished="handover_in_hand")
        # 6. present and wait for the person to take it
        self.step = "present"
        io.move_arm(list(servo), duration_ms=1800)
        confirmed = self.wait_transfer()
        self.mark("pulled", confirmed is not None, **(confirmed or {}))
        if confirmed is None:
            self.retract(holding=True)
            return self.fail("передача не подтверждена: рука не взяла предмет за %.0f с (захват не разжат)" % self.ac["handover_timeout_s"])
        # 7. only now release, then check that the object really is in the hand
        self.step = "release"
        self.release_and_verify(confirmed)

    # ---- sub-steps
    def register_requester(self):
        self.say("запоминаю человека, который дал команду")
        p = self.ctx.perception
        self.people_view()
        p.set_want(people=True, hands=False, pose=False)
        end = time.monotonic() + 20.0
        while time.monotonic() < end:
            self.sleep(0.2)
            snap = p.latest(max_age=1.0)
            if snap is None:
                continue
            c = [t for t in snap.people if t.misses == 0 and t.hits >= 4 and t.dist and t.dist < 3.2 and not t.mono and t.hist is not None]
            if c:
                t = min(c, key=lambda x: abs(x.bearing))
                self.requester = dict(hist=t.hist.copy(), bearing=t.bearing, dist=t.dist, t=time.monotonic())
                self.mark("requester", True, dist=round(t.dist, 2))
                self.say("запомнил: человек на %.2f м" % t.dist)
                return t
        self.mark("requester", False)
        return None

    def find_requester(self):
        """Turn in steps and pick the person whose colour model matches the requester; an unknown person is rejected."""
        io, p = self.io, self.ctx.perception
        pose = list(self.ecfg["people_pose"])
        pose[5] = self.retriever.grip_target() if self.retriever else pose[5]
        io.move_arm(pose, check=False)
        p.set_want(people=True, hands=False, pose=False)
        p.tracker.reset()
        for step in range(self.ac["person_search_turns"]):
            t_end = time.monotonic() + 1.3
            best = None
            while time.monotonic() < t_end:
                self.sleep(0.15)
                snap = p.latest(max_age=1.0)
                if snap is None:
                    continue
                for t in snap.people:
                    if t.misses or t.hits < 3 or not t.dist or t.hist is None:
                        continue
                    d = hist_distance(self.requester["hist"], t.hist)
                    if d <= 0.58 and (best is None or d < best[0]):
                        best = (d, t)
                if best is not None:
                    self.say("человек найден: цвет совпал (расстояние гистограмм %.2f), %.2f м" % (best[0], best[1].dist))
                    return best[1]
            turn_by(io, math.radians(360.0 / self.ac["person_search_turns"]), wz=0.5)
        return None

    def approach(self, requester, track=None):
        io, ctx = self.io, self.ctx
        f = self.ecfg["follow"]
        saved = f["standoff_m"]
        f["standoff_m"] = self.ac["person_standoff_m"]
        runner = FollowRunner(ctx)
        self.runner = runner
        ctx.perception.set_want(people=True, hands=False, pose=False)
        pose = list(self.ecfg["people_pose"])
        if self.retriever is not None:
            pose[5] = self.retriever.grip_target()
        io.move_arm(pose, check=False)
        stable_since, t0 = None, time.monotonic()
        try:
            with base_limits(io, max_vx=f["max_vx"], max_vy=f["max_vy"], max_wz=f["max_wz"], accel=f["accel"],
                             angular_accel=f["angular_accel"]):
                if track is not None:
                    runner.lock.hist = requester["hist"].copy()
                    runner.lock.lock(track, time.monotonic())
                    runner.state = FollowRunner.FOLLOW
                while time.monotonic() - t0 < 50.0 and not self._stop.is_set():
                    snap = ctx.perception.latest(max_age=1.0)
                    now = time.monotonic()
                    runner.tick(snap, now)
                    io.drive(*runner.cmd, hold=0.3)
                    if runner.state == FollowRunner.GAVE_UP:
                        return False
                    tr = runner.target
                    if tr is not None and abs(tr.dist - f["standoff_m"]) <= 0.12 and runner.state == FollowRunner.HOLD:
                        stable_since = stable_since or now
                        if now - stable_since > 1.2:
                            io.stop_base()
                            return True
                    else:
                        stable_since = None
                    self.sleep(0.05)
        finally:
            f["standoff_m"] = saved
            io.stop_base()
        return False

    def wait_hand(self):
        """A hand held out towards the robot: wrist clearly closer to the robot than the shoulder, stable for ~1 s."""
        p = self.ctx.perception
        self.ecfg["detector"]["people_every"] = 3
        p.set_want(people=True, hands=True, pose=True)
        pts, t0 = [], time.monotonic()
        self.say("жду протянутую руку (до %.0f с)" % self.ac["handover_timeout_s"])
        try:
            while time.monotonic() - t0 < self.ac["handover_timeout_s"] and not self._stop.is_set():
                self.sleep(0.2)
                snap = p.latest(max_age=1.0)
                if snap is None:
                    continue
                self._aim_crop(snap)
                cand = self._extended_hand(snap)
                if cand is None:
                    pts = pts[-2:] if pts else []
                    continue
                pts.append(cand)
                pts = pts[-6:]
                if len(pts) >= 5 and float(np.max(np.std(np.array(pts), axis=0))) < 0.03:
                    return np.mean(np.array(pts), axis=0)
        finally:
            self.ecfg["detector"]["people_every"] = 1
        return None

    def _aim_crop(self, snap):
        p = self.ctx.perception
        people = [t for t in snap.people if t.misses == 0 and t.dist is not None]
        if not people:
            p.crop = None
            return
        t = min(people, key=lambda x: x.dist)
        x0, y0, x1, y1 = t.box
        w, h = x1 - x0, y1 - y0
        p.crop = (x0 - 0.8 * w, y0, x1 + 0.8 * w, y1)

    def _extended_hand(self, snap):
        best = None
        for pose in snap.poses:
            for side in ("l", "r"):
                w, s = pose[side + "_wrist"], pose[side + "_shoulder"]
                if w["xyz"] is None or s["xyz"] is None or w["vis"] < 0.5:
                    continue
                reach = s["xyz"][0] - w["xyz"][0]
                if reach < 0.15:
                    continue
                xyz = w["xyz"].copy()
                for h in snap.hands:                              # hand landmarks refine the wrist to the palm centre
                    if h["xyz_base"] is not None and np.linalg.norm(h["pts"][0] - np.array(w["uv"])) < 70:
                        xyz = h["xyz_base"].copy()
                        break
                if best is None or reach > best[0]:
                    best = (reach, xyz)
        return None if best is None else best[1]

    def can_present(self, hand):
        """IK for 'object at the palm': TCP stops 6 cm short of the hand, towards the robot."""
        arm, io = self.io.arm, self.io
        target = np.array(hand, float)
        d = np.hypot(target[0], target[1])
        target[:2] -= 0.06 * target[:2] / max(d, 1e-6)
        if target[2] > 0.56:
            return False, None, target, "рука на высоте %.2f м, манипулятор достаёт не выше ≈0.54 м" % target[2]
        grip = int(round(io.arm_command()[5]))
        for pitch in (0, 15, 30, 45, 60):
            sol, err = arm.ik(target, pitch, io.arm_command())
            if sol is not None:
                full = [int(round(v)) for v in sol] + [grip]
                if arm.within_limits(full) and arm.collision_free(full, self.cfg["arm"]["floor_margin_m"]):
                    return True, full, target, "IK: наклон %d°" % pitch
        return False, None, target, "нет решения IK для точки x=%.2f y=%.2f z=%.2f" % tuple(target)

    def wait_transfer(self):
        """Physical evidence that a person took the object: servo pull (or the gripper stall disappearing) while a hand
        is seen at the gripper. Returns an evidence dict or None on timeout."""
        io, p, arm = self.io, self.ctx.perception, self.io.arm
        p.set_want(people=True, hands=True, pose=False)
        self.ecfg["detector"]["people_every"] = 4
        base = io.measured_arm()
        grip_cmd = io.arm_command()[5]
        gap0 = None if base is None else grip_cmd - base[5]
        t0, pulls, hand_seen_at = time.monotonic(), 0, 0.0
        self.say("предмет предъявлен: жду, пока человек возьмёт его (захват закрыт)")
        try:
            while time.monotonic() - t0 < self.ac["handover_timeout_s"] and not self._stop.is_set():
                self.sleep(0.1)
                snap = p.latest(max_age=1.0)
                now = time.monotonic()
                tcp_px = self._tcp_pixel(snap)
                if snap is not None and tcp_px is not None:
                    for h in snap.hands:
                        c = h["pts"].mean(axis=0)
                        if np.linalg.norm(c - tcp_px) < 140:
                            hand_seen_at = now
                m = io.measured_arm()
                if m is None or base is None:
                    continue
                dev = max(abs(m[i] - io.arm_command()[i]) - abs(base[i] - io.arm_command()[i]) for i in (1, 2, 3))
                gap = grip_cmd - m[5]
                released = gap0 is not None and gap0 > 6.0 and gap < 3.0      # the stalled gripper suddenly closes: object gone
                pulls = pulls + 1 if (dev >= self.ac["tug_deg"] or released) else 0
                if pulls >= 2 and now - hand_seen_at < 1.5:
                    return dict(tug_deg=round(float(dev), 1), gripper_object_removed=bool(released),
                                hand_seen_s_ago=round(now - hand_seen_at, 2))
        finally:
            self.ecfg["detector"]["people_every"] = 1
        return None

    def _tcp_pixel(self, snap):
        if snap is None:
            return None
        K = snap.frame.K
        tcp_cam = self.io.arm.tcp_in_camera(self.io.arm_command())[:3]
        if tcp_cam[2] <= 0.02:
            return np.array([K[0, 2], snap.frame.rgb.shape[0] - 1.0])
        return np.array([K[0, 2] + K[0, 0] * tcp_cam[0] / tcp_cam[2], K[1, 2] + K[1, 1] * tcp_cam[1] / tcp_cam[2]])

    def release_and_verify(self, evidence):
        io = self.io
        r = self.retriever
        r.open_gripper()
        io.wait(0.9)
        # the object must now be seen in the hand (or at least nowhere on the floor / in the gripper)
        seen_in_hand, on_floor = None, False
        for _ in range(4):
            snap = self.ctx.perception.latest(max_age=1.0)
            self.sleep(0.3)
            if snap is None:
                continue
            from ..vision import detect_cubes
            found = detect_cubes(snap.frame, self.cfg["cube"], self.cfg["camera"]) if r.label == "cube" else []
            if found:
                o = found[0]
                tcp_px = self._tcp_pixel(snap)
                hands = [h["pts"].mean(axis=0) for h in snap.hands]
                if any(np.linalg.norm(np.array([o.u, o.v]) - c) < 110 for c in hands):
                    seen_in_hand = True
                    break
                if o.v > snap.frame.rgb.shape[0] * 0.8:
                    on_floor = True
        self.retract(holding=False)
        if on_floor:
            self.mark("handed_over", False, level="dropped")
            return self.fail("предмет упал: после разжатия виден на полу")
        level = "strong (предмет виден в руке)" if seen_in_hand else "weak (рука и рывок подтверждены, предмет в руке камерой не виден)"
        self.mark("handed_over", True, level=level)
        self.result = "передача подтверждена: %s" % level
        self.state = "done"
        self.say(self.result)
        self.finish(True, None, handover_level=level)

    def retract(self, holding):
        pose = list(self.ecfg["people_pose"])
        pose[5] = int(round(self.io.arm_command()[5])) if holding else self.cfg["arm"]["gripper_open"]
        try:
            self.io.move_arm(pose, check=False, duration_ms=1500)
        except Exception as exc:
            self.log("retract failed: %r" % (exc,))

    def overlay(self, img):
        snap = self.ctx.perception.latest()
        if snap is not None:
            draw_people(img, snap.people, None)
            for pose in snap.poses:
                for nm in ("l_wrist", "r_wrist", "l_shoulder", "r_shoulder"):
                    u, v = pose[nm]["uv"]
                    import cv2
                    cv2.circle(img, (int(u), int(v)), 5, (0, 255, 255), -1)
        put_text(img, "E5 step=%s %s" % (self.step, {k: int(v) for k, v in self.steps.items()}), (8, 20), (0, 255, 255))
        return img


def plan_label(plan):
    return plan.get("object") or "cube"
