/* Assembles the docs page: table of contents, sections (lazy init when scrolled near), live poller. */
(function () {
  const toc = $('#toc-links'), content = $('#content');
  SECTIONS.forEach((s, i) => {
    const a = document.createElement('a'); a.href = '#sec-' + s.id; a.textContent = (i + 1) + '. ' + s.title; a.dataset.id = s.id; toc.appendChild(a);
    const el = document.createElement('section'); el.className = 'sec'; el.id = 'sec-' + s.id; el.innerHTML = s.html; content.appendChild(el);
    lazy(el, async () => {
      if (s.id !== 'live') await Promise.resolve();
      try { await s.init(); } catch (e) { console.error(s.id, e); el.insertAdjacentHTML('beforeend', `<div class="note bad">Ошибка раздела: ${String(e)}</div>`); }
      math(el);
    });
  });
  const io = new IntersectionObserver(es => es.forEach(e => { if (e.isIntersecting) document.querySelectorAll('.toc a').forEach(a => a.classList.toggle('on', a.dataset.id === e.target.id.slice(4))); }), { rootMargin: '-30% 0px -60% 0px' });
  document.querySelectorAll('section.sec').forEach(s => io.observe(s));
  Live.start();
})();
