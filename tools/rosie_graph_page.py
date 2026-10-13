"""Rosie's architecture page: a simplified overview diagram + the full live graph (tab).
    python tools/rosie_graph_page.py rosie_graph.svg rosie_graph.json rosie-graph.html   (inputs from tools/rosie_graph.py;
    the overview boxes are hand-placed: edit them when the architecture changes)
"""
import json
import re
import sys

svg_full = open(sys.argv[1], encoding='utf-8').read()
info = json.load(open(sys.argv[2]))
out = sys.argv[3]
svg_full = svg_full[svg_full.index('<svg'):]
svg_full = re.sub(r'<svg width="[^"]*" height="[^"]*"', '<svg id="g" width="100%" height="100%"', svg_full, count=1)
svg_full = re.sub(r'<!--.*?-->', '', svg_full, flags=re.S)

COLS = {'sense': (24, 220), 'understand': (318, 230), 'decide': (622, 230), 'act': (952, 240)}
H = 74


def esc(s):
    return s.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')


def card(stage, y, title, meta, desc, extra=None, h=H, x=None, w=None):
    cx, cw = COLS[stage] if x is None else (x, w)
    lines = [f'<g class="card {stage}" transform="translate({cx},{y})">',
             f'<rect width="{cw}" height="{h}" rx="10"/>',
             f'<text class="t" x="14" y="25">{esc(title)}</text>',
             f'<text class="m" x="14" y="44">{esc(meta)}</text>',
             f'<text class="d" x="14" y="62">{esc(desc)}</text>']
    if extra:
        lines.append(f'<text class="d" x="14" y="79">{esc(extra)}</text>')
    lines.append('</g>')
    return '\n'.join(lines)


def wire(d, drive=False):
    return f'<path class="wire{" drive" if drive else ""}" d="{d}" marker-end="url(#{"ad" if drive else "aw"})"/>'


parts = []
for stage, label in (('sense', 'Sense'), ('understand', 'Understand'), ('decide', 'Decide'), ('act', 'Act')):
    x, w = COLS[stage]
    parts.append(f'<text class="col {stage}" x="{x + 2}" y="44">{label.upper()}</text>')
# Sense
parts += [card('sense', 70, 'Lidar', 'RPLIDAR A1 · 8 scans/s', '360° ring of distances'),
          card('sense', 160, 'IMU', 'BNO055', 'tilt and turn rate'),
          card('sense', 250, 'Depth camera', 'RealSense D435 · 90 fps', 'infrared pair, depth, colour'),
          card('sense', 340, 'Battery', 'INA219', 'pack volts, home when low'),
          card('sense', 430, 'Head camera', 'on the pan-tilt', 'pictures for you and Claude'),
          card('sense', 520, 'Mic + speaker', 'reSpeaker · off on laps', 'her voice and hearing')]
# Understand
parts += [card('understand', 70, 'Visual odometry', 'cuVSLAM · GPU · 45 Hz', 'motion from the camera'),
          card('understand', 160, 'Lidar odometry', 'MOLA', 'motion from the lidar'),
          card('understand', 250, 'Motion fusion', 'EKF · 100 Hz', 'camera + lidar + IMU in one'),
          card('understand', 340, '3D obstacle map', 'nvblox · GPU · 19 Hz', "what the lidar can't see")]
# Decide
parts += [card('decide', 70, 'Where am I?', 'house map + lidar match', 'her place in the house'),
          card('decide', 160, 'Mission controller', 'laps, goals, parking', 'one owner for the job'),
          card('decide', 250, 'Nav2', 'planner + controller', 'plans the route, steers it', 'costmaps: lidar + 3D map', h=90),
          card('decide', 430, 'Your phone / iPad', 'driving page', 'joystick, video, STOP'),
          card('decide', 520, 'Brain', 'Claude + local model', 'talk, and help when stuck')]
# Act
parts += [card('act', 160, 'Command mixer', 'twist_mux', 'you outrank Nav2; STOP wins'),
          card('act', 255, 'Collision guard', '15 cm stop zone', 'lidar + camera obstacles'),
          card('act', 350, 'Motor driver', 'PCA9685', 'pulses for servos and ESC'),
          card('act', 445, 'Wheels', 'drive motor + steering', '4-wheel steering servos'),
          card('act', 540, 'Head', 'pan-tilt servos', 'looks at people and motion')]
# safety band
parts.append('<g class="band"><rect x="24" y="650" width="1168" height="140" rx="12"/>'
             '<text class="col safety" x="40" y="676">WATCHING OVER EVERYTHING</text></g>')
chips = [('Safety gate', 'may she move? how fast?', ''),
         ('Tilt + motion check', 'backs off a tip,', 'stops if the camera is blind'),
         ('Watchdog', 'restarts anything', 'that hangs or goes silent'),
         ('Sensor sleep', 'rests the lidar and laser', 'when she sits idle'),
         ('Relay watchdog', 'hardware: cuts the motors', 'if the computer freezes')]
for i, (t, a, b) in enumerate(chips):
    parts.append(card('safety', 690, t, a, b, x=40 + i * 228, w=216, h=84).replace('class="m"', 'class="d"'))
# wires: Sense -> Understand
parts += [wire('M244 107 C281 107 281 197 318 197'),            # lidar -> lidar odometry
          wire('M244 197 C281 197 281 287 318 287'),            # IMU -> fusion
          wire('M244 287 C281 287 281 107 318 107'),            # depth -> VO
          wire('M244 287 C281 287 281 377 318 377'),            # depth -> 3D map
          wire('M244 467 C440 467 440 467 622 467'),            # head camera -> phone
          wire('M244 557 C440 557 440 557 622 557')]            # mic -> brain
# Understand -> Decide
parts += [wire('M548 287 C585 287 585 300 622 300'),            # fusion -> Nav2
          wire('M548 197 C590 197 595 107 622 107'),            # lidar odometry -> where am I (it reads the scan)
          wire('M737 144 C737 150 737 154 737 160'),            # where am I -> mission (position)
          wire('M737 234 C737 240 737 244 737 250')]            # mission -> Nav2
# to Act: the drive path
parts += [wire('M852 290 C900 290 905 197 952 197', drive=True),  # Nav2 -> mixer
          wire('M852 467 C915 467 915 212 952 212', drive=True),  # phone -> mixer
          wire('M548 377 C700 377 700 377 760 380 C880 384 905 300 952 300'),  # 3D map -> guard
          wire('M1072 234 L1072 255', drive=True),
          wire('M1072 329 L1072 350', drive=True),
          wire('M1072 424 L1072 445', drive=True)]
# safety -> mixer (locks), relay -> motor driver
parts += [wire('M1180 650 C1208 600 1208 210 1192 197'),
          '<text class="note" x="1206" y="420" transform="rotate(90 1206 420)">STOP locks</text>',
          wire('M960 690 C930 660 930 400 952 395'),
          '<text class="note" x="918" y="560" transform="rotate(-90 918 560)">cuts power</text>']

overview = ('<svg id="ov" viewBox="0 0 1216 810" role="img" aria-label="Rosie, simplified: sense, understand, decide, act">'
            '<defs><marker id="aw" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
            '<path d="M0 0 L10 5 L0 10 z" class="ah"/></marker>'
            '<marker id="ad" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
            '<path d="M0 0 L10 5 L0 10 z" class="ahd"/></marker></defs>'
            + '\n'.join(parts) + '</svg>')

shown = set()
for t in info['topics'].values():
    shown |= set(t['pub']) | set(t['sub'])
n_nodes, n_topics = len(shown), len(info['topics'])      # the nodes the graph draws (helpers hidden)

page = r'''<title>Rosie Node Graph</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500&family=IBM+Plex+Sans:wght@400;600&family=Sora:wght@600&display=swap">
<style>
/* Layout: one page, a heading, two tabs; the overview is a 4-stage left-to-right flow over a safety band */
:root {
  --bg: #f3f4f1; --fg: #1c2320; --muted: #5d6762; --line: #d3d8d3; --panel: #ffffff; --paper: #fbfcfa;
  --wire: #8d9792; --drive: #6a4fb3;
  --sense-f: #e4edf9; --sense-s: #7d9fd3; --sense-t: #2f5596;
  --und-f: #e3f2e7; --und-s: #73ad89; --und-t: #2d6a45;
  --dec-f: #faf0d6; --dec-s: #d3a543; --dec-t: #8a6413;
  --act-f: #efe7f9; --act-s: #9a80cf; --act-t: #5a3f99;
  --saf-f: #fbe6e5; --saf-s: #d48a88; --saf-t: #94403d; --band: #f8eeed;
  --display: "Sora", "IBM Plex Sans", system-ui, sans-serif; --body: "IBM Plex Sans", system-ui, sans-serif;
  --mono: "IBM Plex Mono", ui-monospace, monospace;
}
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) {
  --bg: #141917; --fg: #e3e8e4; --muted: #9ba59f; --line: #2b3531; --panel: #1b221f; --paper: #181e1b;
  --wire: #6e7873; --drive: #a58ce6;
  --sense-f: #1a2a40; --sense-s: #4f78b0; --sense-t: #a9c4ef;
  --und-f: #183026; --und-s: #4c9670; --und-t: #9bd6b3;
  --dec-f: #352c17; --dec-s: #b28a31; --dec-t: #e7c776;
  --act-f: #2a2240; --act-s: #8466c2; --act-t: #c9b6f2;
  --saf-f: #391f21; --saf-s: #b46866; --saf-t: #f0b2ae; --band: #241a1b; color-scheme: dark } }
:root[data-theme="dark"] {
  --bg: #141917; --fg: #e3e8e4; --muted: #9ba59f; --line: #2b3531; --panel: #1b221f; --paper: #181e1b;
  --wire: #6e7873; --drive: #a58ce6;
  --sense-f: #1a2a40; --sense-s: #4f78b0; --sense-t: #a9c4ef;
  --und-f: #183026; --und-s: #4c9670; --und-t: #9bd6b3;
  --dec-f: #352c17; --dec-s: #b28a31; --dec-t: #e7c776;
  --act-f: #2a2240; --act-s: #8466c2; --act-t: #c9b6f2;
  --saf-f: #391f21; --saf-s: #b46866; --saf-t: #f0b2ae; --band: #241a1b; color-scheme: dark }
html, body { min-height: 100%; }
body { background: var(--bg); color: var(--fg); font-family: var(--body); padding-inline: 16px; padding-block: 20px 28px;
  box-sizing: border-box; }
main { max-width: 1240px; margin-inline: auto; display: flex; flex-direction: column; gap: 14px; }
h1 { font-family: var(--display); font-size: clamp(1.5rem, 3vw, 2rem); margin: 0; letter-spacing: -.01em; text-wrap: balance; }
.lede { margin: 0; color: var(--muted); max-width: 68ch; line-height: 1.5; }
.tabs { display: flex; gap: 6px; flex-wrap: wrap; }
.tabs button, .bar button, .bar input { font: inherit; font-size: .9rem; color: var(--fg); background: var(--panel);
  border: 1px solid var(--line); border-radius: 999px; padding: 6px 14px; cursor: pointer; }
.tabs button[aria-selected="true"] { background: var(--fg); color: var(--bg); border-color: var(--fg); }
button:focus-visible, input:focus-visible { outline: 2px solid var(--drive); outline-offset: 2px; }
.frame { background: var(--paper); border: 1px solid var(--line); border-radius: 14px; overflow-x: auto; }
#ov { display: block; min-width: 760px; width: 100%; height: auto; font-family: var(--body); }
#ov .card rect { stroke-width: 1.4; }
#ov .t { font-family: var(--display); font-size: 15px; font-weight: 600; fill: var(--fg); }
#ov .m { font-family: var(--mono); font-size: 11.5px; }
#ov .d { font-size: 12.5px; fill: var(--muted); }
#ov .col { font-family: var(--mono); font-size: 12px; letter-spacing: .16em; font-weight: 500; }
#ov .sense rect { fill: var(--sense-f); stroke: var(--sense-s); } #ov .sense .m, #ov .col.sense { fill: var(--sense-t); }
#ov .understand rect { fill: var(--und-f); stroke: var(--und-s); } #ov .understand .m, #ov .col.understand { fill: var(--und-t); }
#ov .decide rect { fill: var(--dec-f); stroke: var(--dec-s); } #ov .decide .m, #ov .col.decide { fill: var(--dec-t); }
#ov .act rect { fill: var(--act-f); stroke: var(--act-s); } #ov .act .m, #ov .col.act { fill: var(--act-t); }
#ov .safety rect { fill: var(--saf-f); stroke: var(--saf-s); } #ov .col.safety { fill: var(--saf-t); }
#ov .band rect { fill: var(--band); stroke: var(--saf-s); stroke-dasharray: 5 4; stroke-width: 1.2; }
#ov .wire { fill: none; stroke: var(--wire); stroke-width: 1.6; }
#ov .wire.drive { stroke: var(--drive); stroke-width: 2.6; }
#ov .ah { fill: var(--wire); } #ov .ahd { fill: var(--drive); }
#ov .note { font-family: var(--mono); font-size: 11px; fill: var(--muted); letter-spacing: .08em; }
.key { display: flex; flex-wrap: wrap; gap: 8px 18px; color: var(--muted); font-size: .88rem; align-items: center; }
.key i { display: inline-block; width: 26px; height: 0; border-top: 2.6px solid var(--drive); vertical-align: middle; margin-right: 6px; }
.key i.g { border-top: 1.6px solid var(--wire); }
.bar { display: flex; flex-wrap: wrap; gap: 8px; align-items: center; }
.bar input { cursor: text; width: 15rem; max-width: 100%; min-width: 0; }
#hint { color: var(--muted); font-size: .86rem; flex: 1 1 18rem; min-width: 0; }
#hint b { color: var(--fg); }
#wrap { height: 72vh; min-height: 380px; background: #fbfcfa; border: 1px solid var(--line); border-radius: 14px;
  overflow: hidden; touch-action: none; cursor: grab; }
#wrap.drag { cursor: grabbing; }
#g { display: block; }
#g .node { cursor: pointer; }
#g.focus .node, #g.focus .edge { opacity: .1; }
#g.focus .on { opacity: 1; }
#g.focus .edge.on path { stroke: #6a4fb3; stroke-width: 2.2; }
#g.focus .edge.on polygon { fill: #6a4fb3; stroke: #6a4fb3; }
#g.focus .node.sel ellipse { stroke: #6a4fb3; stroke-width: 3; }
.foot { color: var(--muted); font-size: .82rem; font-family: var(--mono); }
</style>
<main>
  <h1>Rosie Node Graph</h1>
  <p class="lede">How her software fits together: what she senses, how she works out where she is and what is around her,
    who decides where she goes, and the chain that turns a decision into wheel movement. A safety layer watches all of it.</p>
  <div class="tabs" role="tablist">
    <button id="t1" role="tab" aria-selected="true" aria-controls="p1">Overview</button>
    <button id="t2" role="tab" aria-selected="false" aria-controls="p2">Every node (__NODES__)</button>
  </div>
  <section id="p1" role="tabpanel" aria-labelledby="t1">
    <div class="frame">__OVERVIEW__</div>
    <div class="key" style="margin-top:10px"><span><i></i>the path a drive command takes to the wheels</span>
      <span><i class="g"></i>data</span>
      <span>Rates measured on Rosie on 12–13 Oct 2026.</span></div>
  </section>
  <section id="p2" role="tabpanel" aria-labelledby="t2" hidden>
    <div class="bar">
      <input id="q" list="names" placeholder="Find a node or topic" aria-label="Find a node or topic">
      <datalist id="names"></datalist>
      <button id="zin" aria-label="Zoom in">+</button><button id="zout" aria-label="Zoom out">&minus;</button>
      <button id="fit">Fit</button><button id="clear">Clear</button>
      <span id="hint">Every node and topic. Drag to pan, scroll or +/&minus; to zoom, <b>click a node</b> to see who it talks to.</span>
    </div>
    <div id="wrap" style="margin-top:10px">__FULL__</div>
  </section>
  <p class="foot">Live graph __STAMP__: __NODES__ nodes, __TOPICS__ topics (logging, parameters, tf and one-sided topics hidden).</p>
</main>
<script>
(() => {
  const t1 = document.getElementById('t1'), t2 = document.getElementById('t2');
  const p1 = document.getElementById('p1'), p2 = document.getElementById('p2');
  let ready = false;
  const tab = which => {
    const two = which === 2;
    p1.hidden = two; p2.hidden = !two;
    t1.setAttribute('aria-selected', String(!two)); t2.setAttribute('aria-selected', String(two));
    if (two && !ready) { ready = true; initFull(); }
  };
  t1.onclick = () => tab(1); t2.onclick = () => tab(2);

  function initFull() {
    const svg = document.getElementById('g'), wrap = document.getElementById('wrap');
    const vb0 = svg.getAttribute('viewBox').split(/[ ,]+/).map(Number);
    let vb = vb0.slice();
    const setVB = () => svg.setAttribute('viewBox', vb.join(' '));
    const fit = () => { vb = vb0.slice(); setVB(); };
    const zoom = (k, cx, cy) => {
      const r = wrap.getBoundingClientRect();
      const px = cx === undefined ? .5 : (cx - r.left) / r.width, py = cy === undefined ? .5 : (cy - r.top) / r.height;
      const s = Math.max(vb[2] / r.width, vb[3] / r.height), w = r.width * s, h = r.height * s;
      const x0 = vb[0] - (w - vb[2]) / 2, y0 = vb[1] - (h - vb[3]) / 2, fx = x0 + px * w, fy = y0 + py * h;
      const nw = Math.min(vb0[2] * 1.5, Math.max(80, vb[2] / k)), nh = vb[3] * nw / vb[2];
      vb = [fx - (fx - vb[0]) * nw / vb[2], fy - (fy - vb[1]) * nh / vb[3], nw, nh]; setVB();
    };
    wrap.addEventListener('wheel', e => { e.preventDefault(); zoom(e.deltaY < 0 ? 1.25 : 0.8, e.clientX, e.clientY); }, { passive: false });
    document.getElementById('zin').onclick = () => zoom(1.4);
    document.getElementById('zout').onclick = () => zoom(1 / 1.4);
    document.getElementById('fit').onclick = fit;
    let drag = null, moved = false;
    wrap.addEventListener('pointerdown', e => { drag = { x: e.clientX, y: e.clientY, vb: vb.slice() }; moved = false;
      wrap.setPointerCapture(e.pointerId); wrap.classList.add('drag'); });
    wrap.addEventListener('pointermove', e => { if (!drag) return;
      const r = wrap.getBoundingClientRect(), s = Math.max(vb[2] / r.width, vb[3] / r.height);
      const dx = e.clientX - drag.x, dy = e.clientY - drag.y; if (Math.abs(dx) + Math.abs(dy) > 4) moved = true;
      vb = [drag.vb[0] - dx * s, drag.vb[1] - dy * s, vb[2], vb[3]]; setVB(); });
    const end = () => { drag = null; wrap.classList.remove('drag'); };
    wrap.addEventListener('pointerup', end); wrap.addEventListener('pointercancel', end);
    const nodes = new Map(), edges = [];
    svg.querySelectorAll('g.node').forEach(g => nodes.set(g.querySelector('title').textContent, g));
    svg.querySelectorAll('g.edge').forEach(g => { const [a, b] = g.querySelector('title').textContent.split('->'); edges.push({ a, b, g }); });
    nodes.delete('legend');
    const label = id => id.startsWith('T/') ? id.slice(1) : id;
    const dl = document.getElementById('names');
    [...nodes.keys()].sort().forEach(id => { const o = document.createElement('option'); o.value = label(id); dl.appendChild(o); });
    const hint = document.getElementById('hint'), hint0 = hint.innerHTML;
    const clear = () => { svg.classList.remove('focus'); svg.querySelectorAll('.on,.sel').forEach(x => x.classList.remove('on', 'sel')); hint.innerHTML = hint0; };
    const show = id => {
      clear(); const g = nodes.get(id); if (!g) return;
      svg.classList.add('focus'); g.classList.add('on', 'sel');
      const isTopic = id.startsWith('T/'), topics = new Set(isTopic ? [id] : []), pubs = new Set(), subs = new Set();
      edges.forEach(e => { if (e.a === id) { e.g.classList.add('on'); topics.add(e.b); } if (e.b === id) { e.g.classList.add('on'); topics.add(e.a); } });
      edges.forEach(e => {
        if (topics.has(e.b) && e.b.startsWith('T/')) { e.g.classList.add('on'); nodes.get(e.a)?.classList.add('on'); if (e.a !== id) pubs.add(e.a); }
        if (topics.has(e.a) && e.a.startsWith('T/')) { e.g.classList.add('on'); nodes.get(e.b)?.classList.add('on'); if (e.b !== id) subs.add(e.b); }
      });
      topics.forEach(t => nodes.get(t)?.classList.add('on'));
      const list = s => [...s].map(label).sort().join(', ') || 'none';
      hint.innerHTML = `<b>${label(id)}</b>: ` + (isTopic ? `written by ${list(pubs)}; read by ${list(subs)}.`
        : `hears from ${list(pubs)}; heard by ${list(subs)}.`);
    };
    nodes.forEach((g, id) => g.addEventListener('click', ev => { if (moved) return; ev.stopPropagation(); show(id); }));
    wrap.addEventListener('click', () => { if (!moved) clear(); });
    document.getElementById('clear').onclick = clear;
    document.getElementById('q').addEventListener('change', e => {
      const v = e.target.value.trim(); const id = nodes.has(v) ? v : nodes.has('T' + v) ? 'T' + v : nodes.has('/' + v) ? '/' + v : null;
      if (id) show(id);
    });
    fit();
  }
})();
</script>
'''
page = (page.replace('__OVERVIEW__', overview).replace('__FULL__', svg_full).replace('__STAMP__', '2026-10-13 09:13')
        .replace('__NODES__', str(n_nodes)).replace('__TOPICS__', str(n_topics)))
open(out, 'w', encoding='utf-8').write(page)
print(f'{out}: {len(page) // 1024} KB')
