'use strict';

// ---- helpers ---------------------------------------------------------------
const $ = (id) => document.getElementById(id);
const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
const fmt = (v, digits = 1, unit = '') => (v === null || v === undefined ? '–' : `${v.toFixed(digits)}${unit}`);
const el = (tag, props = {}, text) => {
  const node = Object.assign(document.createElement(tag), props);
  if (text !== undefined) node.textContent = text;  // data is untrusted: never innerHTML
  return node;
};

// Signal colour follows the signal's role, never its rank: command = slot 1,
// feedback = slot 2, any other (context) signals = slots 3+ in profile order.
function slotColors(signals) {
  const colors = {};
  let next = 3;
  for (const s of signals) {
    const slot = s.role === 'command' ? 1 : s.role === 'feedback' ? 2 : Math.min(next++, 8);
    colors[s.key] = css(`--series-${slot}`);
  }
  return colors;
}

const state = { bags: [], analysis: null, axis: 0, shift: false };

// ---- theme -----------------------------------------------------------------
const THEMES = ['auto', 'light', 'dark'];
function applyTheme(theme) {
  if (theme === 'auto') delete document.documentElement.dataset.theme;
  else document.documentElement.dataset.theme = theme;
  $('theme-toggle').textContent = `Theme: ${theme}`;
  try { localStorage.setItem('ma-theme', theme); } catch (e) { /* storage unavailable */ }
  if (state.analysis) renderAxis();
}
$('theme-toggle').addEventListener('click', () => {
  const cur = document.documentElement.dataset.theme || 'auto';
  applyTheme(THEMES[(THEMES.indexOf(cur) + 1) % THEMES.length]);
});
window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => { if (state.analysis) renderAxis(); });
{
  // #theme=light|dark in the URL wins (handy for screenshots), then the saved choice.
  const fromHash = new URLSearchParams(location.hash.slice(1)).get('theme');
  let saved = null;
  try { saved = localStorage.getItem('ma-theme'); } catch (e) { /* storage unavailable */ }
  applyTheme(THEMES.includes(fromHash) ? fromHash : THEMES.includes(saved) ? saved : 'auto');
}

// ---- data ------------------------------------------------------------------
async function getJSON(url) {
  const res = await fetch(url);
  const body = await res.json().catch(() => ({ error: `HTTP ${res.status}` }));
  if (!res.ok || body.error) throw new Error(body.error || `HTTP ${res.status}`);
  return body;
}

async function loadBags() {
  const data = await getJSON('/api/bags');
  state.bags = data.bags;
  const sel = $('bag-select');
  sel.replaceChildren();
  for (const b of data.bags) {
    const label = b.readable ? `${b.name}  ·  ${b.modified.replace('T', ' ')}${b.profile ? '' : '  ·  no matching profile'}`
                             : `${b.name}  ·  can't be read here`;
    const opt = el('option', { value: b.name, disabled: !b.readable }, label);
    if (!b.readable) opt.title = b.error || '';
    sel.append(opt);
  }
  const fromHash = new URLSearchParams(location.hash.slice(1)).get('bag');
  const first = data.bags.find((b) => b.name === fromHash && b.readable) ||
                data.bags.find((b) => b.readable && b.profile) || data.bags.find((b) => b.readable);
  if (first) { sel.value = first.name; analyze(); }
  else showError(`No readable recordings in ${data.experiment_dir}. Record one with record_motion.`);
}

async function analyze() {
  const bag = $('bag-select').value;
  if (!bag) return;
  const params = new URLSearchParams({ bag });
  history.replaceState(null, '', `#bag=${encodeURIComponent(bag)}`);

  $('content').classList.add('loading');  // keep the previous render, dimmed
  try {
    state.analysis = await getJSON(`/api/analysis?${params}`);
    $('error-card').hidden = true;
    const moved = state.analysis.axes.findIndex((a) => a.moved);
    state.axis = Math.min(state.axis, state.analysis.axes.length - 1);
    if (!state.analysis.axes[state.axis]?.moved && moved >= 0) state.axis = moved;
    renderSummary();
    renderAxis();
  } catch (e) {
    showError(e.message);
  } finally {
    $('content').classList.remove('loading');
  }
}

function showError(message) {
  $('error-text').textContent = message;
  $('error-card').hidden = false;
}

$('bag-select').addEventListener('change', analyze);

// ---- summary ---------------------------------------------------------------
function renderSummary() {
  const a = state.analysis;
  $('summary-card').hidden = false;
  $('bag-meta').textContent = `${a.bag} · profile ${a.profile.name} · ${fmt(a.duration_s, 1, ' s')} recording`;
  const none = a.summary.typical_motion_ms === null;
  $('hero-start').textContent = none ? 'No moves' : fmt(a.summary.typical_start_ms, 1, ' ms');
  $('hero-motion').textContent = none ? 'No moves' : fmt(a.summary.typical_motion_ms, 1, ' ms');
  $('hero-note').textContent = none
    ? 'The command signal never moved in this recording.'
    : `Typical (median) per-move values across ${a.summary.moved_axes.length} moved axes (${a.summary.moved_axes.join(', ')}). ` +
      `Command: ${a.profile.command}. Feedback: ${a.profile.feedback}.`;

  const tbody = $('summary-table').querySelector('tbody');
  tbody.replaceChildren();
  a.axes.forEach((ax, i) => {
    const o = ax.overall, s = ax.shift;
    const means = ax.moves.map((m) => m.mean_ms).filter((v) => v !== null);
    const tr = el('tr', { tabIndex: 0 });
    tr.setAttribute('aria-selected', String(i === state.axis));
    const cells = [
      [ax.joint ? `${ax.label} (${ax.joint})` : ax.label, ''],
      [ax.moved ? String(ax.moves.length) : '0', 'num'],
      [o ? fmt(o.typical_start_ms, 1, ' ms') : '–', 'num'],
      [o ? fmt(o.typical_motion_ms, 1, ' ms') : '–', 'num'],
      [means.length ? `${Math.min(...means).toFixed(0)} – ${Math.max(...means).toFixed(0)} ms` : '–', 'num'],
      [s ? `${fmt(s.rms_unshifted_deg, 3)}° → ${fmt(s.rms_shifted_deg, 3)}°` : '–', 'num'],
      [ax.moved ? ax.flags.map((f) => f.text).join('; ') : 'Not moved in this recording', ax.moved ? '' : 'muted'],
    ];
    for (const [text, cls] of cells) tr.append(el('td', { className: cls }, text));
    const select = () => { state.axis = i; renderSummary(); renderAxis(); };
    tr.addEventListener('click', select);
    tr.addEventListener('keydown', (ev) => { if (ev.key === 'Enter' || ev.key === ' ') { ev.preventDefault(); select(); } });
    tbody.append(tr);
  });
}

// ---- axis charts -----------------------------------------------------------
function baseLayout(extra = {}) {
  const grid = css('--grid'), axis = css('--axis'), muted = css('--text-secondary');
  // A fresh object per axis: sharing one nested `title` object would make setting
  // the y title overwrite the x title.
  const axisStyle = () => ({ gridcolor: grid, linecolor: axis, zeroline: false, tickfont: { color: muted },
                             title: { text: '', font: { color: muted, size: 12 } }, showline: true, ticks: '' });
  return Object.assign({
    paper_bgcolor: css('--surface-1'), plot_bgcolor: css('--surface-1'),
    font: { family: 'system-ui, -apple-system, "Segoe UI", sans-serif', color: css('--text-primary'), size: 12 },
    margin: { l: 64, r: 16, t: 8, b: 44 },
    xaxis: axisStyle(), yaxis: axisStyle(),
    legend: { orientation: 'h', x: 0, y: 1.02, yanchor: 'bottom', font: { color: muted } },
    hoverlabel: { bgcolor: css('--surface-1'), bordercolor: axis, font: { color: css('--text-primary') } },
  }, extra);
}
const plotConfig = { responsive: true, displaylogo: false, modeBarButtonsToRemove: ['lasso2d', 'select2d', 'autoScale2d'] };

function renderAxis() {
  const a = state.analysis;
  if (!a) return;
  const ax = a.axes[state.axis];
  $('axis-card').hidden = false;
  $('axis-title').textContent = ax.joint ? `Axis ${ax.label} (${ax.joint})` : `Axis ${ax.label}`;

  const tabs = $('axis-tabs');
  tabs.replaceChildren();
  a.axes.forEach((other, i) => {
    const b = el('button', { type: 'button', className: other.moved ? '' : 'idle' }, other.label);
    b.setAttribute('role', 'tab');
    b.setAttribute('aria-selected', String(i === state.axis));
    if (!other.moved) b.title = 'Not moved in this recording';
    b.addEventListener('click', () => { state.axis = i; renderSummary(); renderAxis(); });
    tabs.append(b);
  });

  const colors = slotColors(ax.signals);
  renderPosition(ax, colors);
  renderMoves(ax);
  renderMovesTable(ax);
}

function moveBands(ax) {
  return ax.moves.map((m) => ({ type: 'rect', xref: 'x', yref: 'paper', x0: m.start_s, x1: m.end_s, y0: 0, y1: 1,
                                fillcolor: css('--wash'), line: { width: 0 }, layer: 'below' }));
}

function renderPosition(ax, colors) {
  const toggle = $('shift-toggle');
  const delay = ax.shift ? ax.shift.delay_ms : null;
  toggle.disabled = delay === null;
  toggle.checked = state.shift && delay !== null;
  $('shift-label').textContent = delay === null ? 'Shift feedback back by the typical motion delay (no moves)'
                                                 : `Shift feedback back by the typical motion delay (${delay.toFixed(1)} ms)`;
  const shifted = toggle.checked;
  $('position-note').textContent = ax.moved
    ? `Shaded: the ${ax.moves.length} detected moves. ` + (shifted
        ? `Feedback drawn ${delay.toFixed(1)} ms earlier - where it overlaps the command, the lag is a pure time delay.`
        : 'Hover anywhere to read every signal at that time.') + (ax.decimated ? ' Long recording: points thinned for display.' : '')
    : 'This axis did not move in this recording.';

  const traces = ax.signals.map((s) => {
    const shift = shifted && s.role === 'feedback' ? delay / 1000 : 0;
    return {
      type: 'scatter', mode: 'lines', name: shift ? `${s.label}, shifted ${delay.toFixed(0)} ms` : s.label,
      x: shift ? s.t.map((t) => t - shift) : s.t, y: s.p,
      line: { color: colors[s.key], width: 2 },
      hovertemplate: '%{y:.3f}°<extra>%{fullData.name}</extra>',
    };
  });
  const layout = baseLayout({ shapes: moveBands(ax), hovermode: 'x unified' });
  layout.xaxis.title.text = 'Time since recording start (s)';
  layout.yaxis.title.text = 'Position (deg)';
  layout.xaxis.hoverformat = '.3f';
  layout.xaxis.range = [0, state.analysis.duration_s];
  Plotly.react('chart-position', traces, layout, plotConfig);
}
$('shift-toggle').addEventListener('change', (ev) => { state.shift = ev.target.checked; renderAxis(); });

function renderMoves(ax) {
  const moves = ax.moves.filter((m) => m.mean_ms !== null || m.start_ms !== null);
  const typical = ax.overall ? ax.overall.typical_motion_ms : null;
  const x = moves.map((m) => `Move ${m.index}`);
  const ring = { color: css('--surface-1'), width: 2 };
  const motionColor = css('--series-1'), startColor = css('--series-3');
  const traces = [
    {
      type: 'scatter', mode: 'markers', name: 'Start delay', x, y: moves.map((m) => m.start_ms),
      marker: { size: 10, symbol: 'diamond', color: startColor, line: ring },
      customdata: moves.map((m) => [m.travel_deg, m.start_s]),
      hovertemplate: '<b>%{y:.1f} ms</b> start<br>travel %{customdata[0]:.1f}° · starts %{customdata[1]:.2f} s<extra>%{x}</extra>',
    },
    {
      type: 'scatter', mode: 'markers', name: 'Motion delay', x, y: moves.map((m) => m.mean_ms),
      error_y: { type: 'data', symmetric: false, array: moves.map((m) => (m.max_ms ?? 0) - (m.mean_ms ?? 0)),
                 arrayminus: moves.map((m) => (m.mean_ms ?? 0) - (m.min_ms ?? 0)), color: motionColor, thickness: 2, width: 6 },
      marker: { size: 10, color: motionColor, line: ring },
      customdata: moves.map((m) => [m.n, m.min_ms, m.max_ms]),
      hovertemplate: '<b>%{y:.1f} ms</b> motion mean<br>range %{customdata[1]:.1f} – %{customdata[2]:.1f} ms · ' +
                     '%{customdata[0]} samples<extra>%{x}</extra>',
    },
  ];
  const layout = baseLayout({ hovermode: 'closest' });
  layout.yaxis.title.text = 'Delay (ms)';
  layout.xaxis.type = 'category';
  // From zero, with headroom, so no point or whisker sits on the frame.
  const top = Math.max(0, ...moves.flatMap((m) => [m.max_ms ?? 0, m.start_ms ?? 0]));
  layout.yaxis.range = [0, (top || 1) * 1.15];
  if (typical !== null) {
    layout.shapes = [{ type: 'line', xref: 'paper', x0: 0, x1: 1, y0: typical, y1: typical,
                       line: { color: css('--text-muted'), width: 1 } }];
    // Between the first two moves' points (category positions 0 and 1), where none sits.
    layout.annotations = [{ xref: moves.length > 1 ? 'x' : 'paper', x: 0.5, y: typical, xanchor: 'center',
                            yanchor: 'bottom', showarrow: false,
                            text: `typical motion ${typical.toFixed(1)} ms`, font: { color: css('--text-secondary'), size: 11 } }];
  }
  if (!moves.length) {
    layout.annotations = [{ xref: 'paper', yref: 'paper', x: 0.5, y: 0.5, showarrow: false,
                            text: 'No moves on this axis', font: { color: css('--text-muted') } }];
  }
  Plotly.react('chart-moves', moves.length ? traces : [], layout, plotConfig);
}

function renderMovesTable(ax) {
  const tbody = $('moves-table').querySelector('tbody');
  tbody.replaceChildren();
  for (const m of ax.moves) {
    const tr = el('tr');
    for (const v of [String(m.index), fmt(m.start_s, 3), fmt(m.end_s, 3), fmt(m.travel_deg, 2), fmt(m.start_ms, 2), String(m.n),
                     fmt(m.mean_ms, 2), fmt(m.min_ms, 2), fmt(m.max_ms, 2)]) {
      tr.append(el('td', { className: 'num' }, v));
    }
    tbody.append(tr);
  }
  if (!ax.moves.length) {
    const tr = el('tr');
    tr.append(el('td', { colSpan: 9, className: 'muted' }, 'No moves on this axis.'));
    tbody.append(tr);
  }
}

loadBags().catch((e) => showError(e.message));
