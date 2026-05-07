const RESULTS_URL  = 'results.json';
const TRAINLOG_URL = 'data/train_log.json';
const REFRESH_MS   = 5000;

const RUN_COLORS = [
  '#4ecb71', '#5ba3e8', '#e8a830', '#3ecaa5',
  '#c87aad', '#e87a5b', '#9a8fe0', '#e8d85b',
];
export function initResultsPanel(openBtn, overlay, closeBtn) {
  let charts    = [];
  let timer     = null;
  let lastCount = 0;

  openBtn.addEventListener('click', () => {
    overlay.classList.remove('hidden');
    load();
    timer = setInterval(load, REFRESH_MS);
  });

  closeBtn.addEventListener('click', close);
  overlay.addEventListener('click', (e) => { if (e.target === overlay) close(); });

  function close() {
    overlay.classList.add('hidden');
    clearInterval(timer);
    timer = null;
  }

  async function load() {
    loadTrainLog();

    let runs;
    try {
      const res = await fetch(`${RESULTS_URL}?_=${Date.now()}`);
      if (!res.ok) { showEmpty('Нет данных. Запусти runner с --out results.json'); return; }
      runs = await res.json();
    } catch {
      showEmpty('Нет данных. Запусти runner с --out results.json');
      return;
    }

    if (!Array.isArray(runs) || runs.length === 0) {
      showEmpty('results.json пуст.');
      return;
    }

    if (runs.length === lastCount) return;
    lastCount = runs.length;

    buildTable(runs);
  }

  async function loadTrainLog() {
    try {
      const res = await fetch(`${TRAINLOG_URL}?_=${Date.now()}`);
      if (!res.ok) return;
      const log = await res.json();
      if (!Array.isArray(log) || log.length === 0) return;
      document.getElementById('trainLogWrap').classList.remove('hidden');
      const canvas = document.getElementById('chartTrainLog');
      if (charts._trainClickHandler) canvas.removeEventListener('click', charts._trainClickHandler);
      if (charts._trainChart) charts._trainChart.destroy();
      charts._trainChart = buildTrainLogChart(log);
      charts._trainClickHandler = () => { close(); window.dispatchEvent(new CustomEvent('startReplay')); };
      canvas.addEventListener('click', charts._trainClickHandler);
      canvas.style.cursor = 'pointer';
    } catch {}
  }

  function showEmpty(msg) {
    document.getElementById('chartEmpty').textContent = msg;
    document.getElementById('chartEmpty').classList.remove('hidden');
    document.getElementById('runsTable').innerHTML = '';
  }
}


function fmtElapsed(sec) {
  if (sec == null) return '—';
  const h = Math.floor(sec / 3600);
  const m = Math.floor((sec % 3600) / 60);
  return h > 0 ? `${h}h ${m}m` : `${m}m`;
}

function buildTable(runs) {
  document.getElementById('chartEmpty').classList.add('hidden');
  const tbody = runs.map((run, i) => {
    const col = RUN_COLORS[i % RUN_COLORS.length];
    const s   = run.summary;
    return `<tr>
      <td><span class="run-dot" style="background:${col}"></span>${run.label ?? run.policy}</td>
      <td>${run.params.ndrones}v${run.params.nanti}</td>
      <td>${run.records.length}</td>
      <td>${s.mean_reward}</td>
      <td>${s.hit_rate}</td>
      <td>${s.intercept_rate}</td>
      <td>${s.mean_time}s</td>
      <td>${fmtElapsed(run.elapsed_sec)}</td>
    </tr>`;
  }).join('');

  document.getElementById('runsTable').innerHTML = `
    <table class="runs-table">
      <thead><tr>
        <th>Policy</th><th>Params</th><th>Ep</th>
        <th>Reward</th><th>Hit</th><th>Int</th><th>Ep time</th><th>Train time</th>
      </tr></thead>
      <tbody>${tbody}</tbody>
    </table>`;
}

function buildTrainLogChart(log) {
  const ctx    = document.getElementById('chartTrainLog').getContext('2d');
  const labels = log.map((r) => r.epoch);

  const chart = new window.Chart(ctx, {
    type: 'line',
    data: {
      labels,
      datasets: [
        {
          label:           'Hit rate',
          data:            log.map((r) => r.eval_hit      ?? null),
          borderColor:     '#4ecb71',
          backgroundColor: 'rgba(78,203,113,0.08)',
          borderWidth:     2,
          pointRadius:     3,
          tension:         0.3,
          spanGaps:        true,
          yAxisID:         'y',
        },
        {
          label:           'Intercept rate',
          data:            log.map((r) => r.eval_intercept ?? null),
          borderColor:     '#e04b4b',
          backgroundColor: 'transparent',
          borderWidth:     2,
          pointRadius:     3,
          tension:         0.3,
          spanGaps:        true,
          yAxisID:         'y',
        },
        {
          label:           'Reward',
          data:            log.map((r) => r.eval_reward   ?? null),
          borderColor:     '#e8a830',
          backgroundColor: 'transparent',
          borderWidth:     2,
          pointRadius:     3,
          tension:         0.3,
          spanGaps:        true,
          yAxisID:         'y2',
        },
        {
          label:           'Entropy',
          data:            log.map((r) => r.entropy       ?? null),
          borderColor:     '#9a8fe0',
          backgroundColor: 'transparent',
          borderWidth:     1,
          pointRadius:     0,
          tension:         0,
          spanGaps:        true,
          yAxisID:         'y3',
          hidden:          true,
        },
      ],
    },
    options: {
      responsive:          true,
      maintainAspectRatio: false,
      animation:           false,
      plugins: {
        legend: { labels: { color: '#c8d8c0', font: { family: 'Share Tech Mono', size: 13 } } },
        title:  { display: true, text: 'Training progress', color: '#4ecb71', font: { family: 'Share Tech Mono', size: 14 } },
        tooltip: { titleFont: { size: 14 }, bodyFont: { size: 13 } },
      },
      scales: {
        x:  { ticks: { color: 'rgba(200,216,192,0.6)', font: { family: 'Share Tech Mono', size: 12 } }, grid: { color: 'rgba(80,200,100,0.08)' } },
        y:  { min: 0, max: 1, ticks: { color: 'rgba(200,216,192,0.6)', font: { size: 12 } }, grid: { color: 'rgba(80,200,100,0.08)' }, title: { display: true, text: 'rate', color: 'rgba(200,216,192,0.6)', font: { size: 12 } } },
        y2: { position: 'right', ticks: { color: 'rgba(232,168,48,0.7)', font: { size: 12 } }, grid: { drawOnChartArea: false }, title: { display: true, text: 'reward', color: 'rgba(232,168,48,0.7)', font: { size: 12 } } },
        y3: { display: false },
      },
    },
  });

  return chart;
}

function chartOptions(title) {
  return {
    responsive:          true,
    maintainAspectRatio: false,
    animation:           false,
    plugins: {
      legend: { labels: { color: '#c8d8c0', font: { family: 'Share Tech Mono', size: 13 } } },
      title:  { display: true, text: title, color: '#4ecb71', font: { family: 'Share Tech Mono', size: 14 } },
      tooltip: { titleFont: { size: 14 }, bodyFont: { size: 13 } }
    },
    scales: {
      x: { ticks: { color: 'rgba(200,216,192,0.6)', font: { size: 12 } }, grid: { color: 'rgba(80,200,100,0.08)' } },
      y: { ticks: { color: 'rgba(200,216,192,0.6)', font: { size: 12 } }, grid: { color: 'rgba(80,200,100,0.08)' } },
    },
  };
}
