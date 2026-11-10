'use strict';

(() => {
  const TOKEN = document.querySelector('meta[name="panel-token"]').getAttribute('content');
  const AXIS_COUNT = 5;
  const PLAN_SCHEMA = 'slaihand_joint_plan/2';
  const MAX_WAYPOINTS = 12;

  const $ = (id) => document.getElementById(id);
  const S = {
    data: null,
    connected: false,
    ticket: null,
    dirty: false,
    rows: [],
    axisCells: [],
    polling: false,
    rendered: { preview: null, operation: null, events: null },
    teaching: { lastId: null, previewSignature: null, ticket: null, selected: null },
    capture: { ticket: null, previewSignature: null, resultSignature: null },
    calibrationGroup: null,
    lastPollError: null,
    pollMs: null,
  };
  const CONFIRMATIONS = ['supported', 'clearance_and_local_power_cut', 'native_output_not_hardware_clamp'];

  // ---------------------------------------------------------------- helpers
  function el(tag, cls, text) {
    const node = document.createElement(tag);
    if (cls) node.className = cls;
    if (text !== undefined) node.textContent = text;
    return node;
  }

  function finite(value) {
    return typeof value === 'number' && Number.isFinite(value);
  }

  function view(value, digits) {
    return finite(value) ? Number(value).toFixed(digits === undefined ? 2 : digits) : '—';
  }

  function viewInt(value) {
    return Number.isInteger(value) ? String(value) : '—';
  }

  function clockOf(seconds) {
    if (!finite(seconds)) return '—';
    return new Date(seconds * 1000).toLocaleTimeString('zh-CN', { hour12: false });
  }

  function dateOf(seconds) {
    if (!finite(seconds)) return '—';
    return new Date(seconds * 1000).toLocaleString('zh-CN', { hour12: false });
  }

  function bool(value, yes, no) {
    return value === true ? (yes || '是') : value === false ? (no || '否') : '—';
  }

  function setMessage(text, kind, quiet) {
    const node = $('message');
    node.textContent = text;
    node.className = 'message' + (kind ? ' ' + kind : '');
    if (!quiet) S.lastPollError = null;
  }

  function describeError(error) {
    if (error && error.payload && error.payload.code) {
      const payload = error.payload;
      const preflight = payload.calibration_preflight || payload.j5_preflight || {};
      const reasons = preflight.reasons || [];
      const message = payload.message || '请求被拒绝';
      const alreadyListed = reasons.length > 0
        && reasons.every((reason) => message.includes(reason.split(':')[0]));
      const details = reasons.length && !alreadyListed
        ? '\n逐项原因：' + reasons.join('\n') : '';
      const observed = preflight.observations
        ? '\n观察值：' + JSON.stringify(preflight.observations, null, 2) : '';
      return payload.code + '：' + message + details + observed;
    }
    return (error && error.message) || '未知错误';
  }

  async function api(method, path, body) {
    const options = { method: method, credentials: 'same-origin', headers: { 'X-Panel-Token': TOKEN } };
    if (body !== undefined) {
      options.headers['Content-Type'] = 'application/json';
      options.body = JSON.stringify(body);
    }
    const response = await fetch(path, options);
    let payload = null;
    try {
      payload = await response.json();
    } catch (error) {
      payload = { message: '响应不是 JSON（HTTP ' + response.status + '）' };
    }
    if (!response.ok) {
      const failure = new Error(payload.message || ('HTTP ' + response.status));
      failure.payload = payload;
      failure.status = response.status;
      throw failure;
    }
    return payload;
  }

  function confirmations() {
    return {
      supported: $('ack-supported').checked === true,
      clearance_and_local_power_cut: $('ack-clearance').checked === true,
      native_output_not_hardware_clamp: $('ack-native').checked === true,
      free_static: $('ack-free-static').checked === true,
    };
  }

  function allConfirmed() {
    const state = confirmations();
    return CONFIRMATIONS.every((key) => state[key] === true)
      && ($('execution-mode').value !== 'calibration' || state.free_static);
  }

  // ----------------------------------------------------------------- editor
  function currentMode() {
    const checked = document.querySelector('input[name="coords"]:checked');
    return checked ? checked.value : 'relative_start_deg';
  }

  function numberInput(value, step, min) {
    const input = document.createElement('input');
    input.type = 'number';
    input.step = step;
    if (min !== undefined) input.min = String(min);
    input.value = value;
    return input;
  }

  function addRow(values, move, dwell) {
    const tr = el('tr');
    const indexCell = el('td');
    tr.appendChild(indexCell);
    const inputs = [];
    for (let axis = 0; axis < AXIS_COUNT; axis += 1) {
      const cell = el('td');
      const input = numberInput(Number(values[axis] || 0).toFixed(3), '0.1');
      input.addEventListener('input', markDirty);
      cell.appendChild(input);
      tr.appendChild(cell);
      inputs.push(input);
    }
    const moveCell = el('td');
    const moveInput = numberInput(Number(move || 4).toFixed(2), '0.5', 0.1);
    moveInput.addEventListener('input', markDirty);
    moveCell.appendChild(moveInput);
    tr.appendChild(moveCell);
    const dwellCell = el('td');
    const dwellInput = numberInput(Number(dwell === undefined ? 1 : dwell).toFixed(2), '0.5', 0.5);
    dwellInput.addEventListener('input', markDirty);
    dwellCell.appendChild(dwellInput);
    tr.appendChild(dwellCell);
    const actionCell = el('td');
    const remove = el('button', null, '删除');
    remove.type = 'button';
    remove.addEventListener('click', () => removeRow(tr));
    actionCell.appendChild(remove);
    tr.appendChild(actionCell);
    $('waypoints-body').appendChild(tr);
    S.rows.push({ tr: tr, indexCell: indexCell, inputs: inputs, move: moveInput, dwell: dwellInput });
    renumber();
  }

  function removeRow(tr) {
    if (S.rows.length <= 1) {
      setMessage('至少保留一个路点。', 'warn');
      return;
    }
    const index = S.rows.findIndex((row) => row.tr === tr);
    if (index < 0) return;
    tr.remove();
    S.rows.splice(index, 1);
    renumber();
    markDirty();
  }

  function renumber() {
    S.rows.forEach((row, index) => {
      row.indexCell.textContent = String(index + 1);
    });
  }

  function resetEditor() {
    $('waypoints-body').textContent = '';
    S.rows = [];
  }

  function markDirty() {
    S.dirty = true;
    $('editor-hint').textContent = '计划已修改：旧预览与票根不再可用，请在服务器重新预览。';
    updateButtons();
  }

  function markClean() {
    S.dirty = false;
    $('editor-hint').textContent = '预览与当前编辑一致；执行前仍需三项确认。';
    updateButtons();
  }

  function applyPlan(document, source, calibrationGroup = null) {
    S.calibrationGroup = calibrationGroup || null;
    if (!document || typeof document !== 'object' || Array.isArray(document)) {
      throw new Error('计划必须是 JSON 对象');
    }
    if (document.schema !== PLAN_SCHEMA) {
      throw new Error('这是旧四轴计划或未声明版本的计划（schema=' + (document.schema || '缺失')
        + '），不能当作五轴计划载入；请显式保存为 ' + PLAN_SCHEMA + ' 并补齐 J5 角度');
    }
    if (document.coordinates !== 'absolute_deg' && document.coordinates !== 'relative_start_deg') {
      throw new Error('coordinates 必须是 absolute_deg 或 relative_start_deg');
    }
    if (!Array.isArray(document.waypoints) || document.waypoints.length < 1 || document.waypoints.length > MAX_WAYPOINTS) {
      throw new Error('需要 1～' + MAX_WAYPOINTS + ' 个路点');
    }
    document.waypoints.forEach((row, index) => {
      if (!row || typeof row !== 'object' || Array.isArray(row)) throw new Error('第' + (index + 1) + '点必须是对象');
      if (!Array.isArray(row.degrees) || row.degrees.length !== AXIS_COUNT) throw new Error('第' + (index + 1) + '点需要 J1～J5 五个角度');
      row.degrees.forEach((value) => {
        if (typeof value !== 'number' || !Number.isFinite(value)) throw new Error('第' + (index + 1) + '点角度必须是有限数值');
      });
      if (typeof row.move_s !== 'number' || !Number.isFinite(row.move_s)) throw new Error('第' + (index + 1) + '点移动时间无效');
      if (typeof row.dwell_s !== 'number' || !Number.isFinite(row.dwell_s)) throw new Error('第' + (index + 1) + '点停留时间无效');
    });
    (document.coordinates === 'absolute_deg' ? $('mode-absolute') : $('mode-relative')).checked = true;
    resetEditor();
    document.waypoints.forEach((row) => addRow(row.degrees, row.move_s, row.dwell_s));
    S.ticket = null;
    markDirty();
    setMessage('已载入计划（仅数据，不执行）：' + (source || '未命名') + '。请先在服务器预览。', 'ok');
  }

  function planFromEditor() {
    const waypoints = [];
    for (let index = 0; index < S.rows.length; index += 1) {
      const row = S.rows[index];
      const degrees = row.inputs.map((input) => (input.value.trim() === '' ? NaN : Number(input.value)));
      if (degrees.some((value) => !Number.isFinite(value))) {
        return { ok: false, message: '第' + (index + 1) + '点有无效角度' };
      }
      const move = row.move.value.trim() === '' ? NaN : Number(row.move.value);
      const dwell = row.dwell.value.trim() === '' ? NaN : Number(row.dwell.value);
      if (!Number.isFinite(move) || move <= 0) return { ok: false, message: '第' + (index + 1) + '点移动时间必须大于 0' };
      if (!Number.isFinite(dwell) || dwell < 0.5) return { ok: false, message: '第' + (index + 1) + '点停留时间至少 0.5 秒' };
      waypoints.push({ degrees: degrees, move_s: move, dwell_s: dwell });
    }
    return { ok: true, document: { schema: PLAN_SCHEMA, coordinates: currentMode(), waypoints: waypoints } };
  }

  // ------------------------------------------------------------------ actions
  async function readCurrent() {
    if (!S.data || !Array.isArray(S.data.axes)) {
      setMessage('还没有五轴状态，不能读取当前位置。', 'warn');
      return;
    }
    const values = S.data.axes.map((axis) => axis.actual_deg);
    if (values.length !== AXIS_COUNT || values.some((value) => !finite(value))) {
      setMessage('没有完整的五轴反馈（—/失联），不能读取当前位置，也不以 0 代替 J5。', 'error');
      return;
    }
    S.rows.forEach((row) => {
      row.inputs.forEach((input, axis) => {
        input.value = currentMode() === 'absolute_deg' ? values[axis].toFixed(3) : '0.000';
      });
    });
    markDirty();
    setMessage(currentMode() === 'absolute_deg'
      ? '已把当前实际角度填入全部路点目标（软件绝对角度）。没有使能，也没有运动。'
      : '相对本次起点模式：0 表示当前位置，已填入 0。没有使能，也没有运动。', 'ok');
  }

  async function doPreview() {
    const parsed = planFromEditor();
    if (!parsed.ok) {
      setMessage(parsed.message, 'error');
      return;
    }
    try {
      const response = await api('POST', '/api/preview', {
        document: parsed.document, execution_mode: $('execution-mode').value,
        calibration_group: S.calibrationGroup,
      });
      S.ticket = response.ticket.id;
      S.dirty = false;
      markClean();
      setMessage('服务器校验通过：' + response.preview.waypoints.length + ' 点、'
        + Number(response.preview.total_s).toFixed(1) + ' 秒；票根 ' + response.ticket.id.slice(0, 8)
        + '…。这是预览，没有请求使能或轨迹。', 'ok');
      await poll();
    } catch (error) {
      S.ticket = null;
      setMessage('预览被拒绝：' + describeError(error), 'error');
    }
  }

  async function doExecute() {
    const parsed = planFromEditor();
    if (!parsed.ok) {
      setMessage(parsed.message, 'error');
      return;
    }
    if (!S.ticket) {
      setMessage('没有有效票根：请先在服务器预览。', 'warn');
      return;
    }
    if (S.dirty) {
      setMessage('计划在预览后被修改：请重新预览。', 'warn');
      return;
    }
    if (!allConfirmed()) {
      setMessage('三项确认必须全部勾选；服务端同样会强制校验。', 'warn');
      return;
    }
    try {
      const response = await api('POST', '/api/execute', {
        ticket: S.ticket, document: parsed.document, confirmations: confirmations(),
        execution_mode: $('execution-mode').value, calibration_group: S.calibrationGroup,
      });
      S.ticket = null;
      markDirty();
      setMessage('执行请求已受理（不是已完成）：结果、原始错误码与逐点判据见“操作结果”。', 'warn');
      await poll();
    } catch (error) {
      setMessage('执行被拒绝：' + describeError(error), 'error');
      await poll();
    }
  }

  async function doHold() {
    if (!allConfirmed()) {
      setMessage('有限保持同样需要三项确认（服务端强制）。', 'warn');
      return;
    }
    try {
      const response = await api('POST', '/api/hold', { confirmations: confirmations() });
      setMessage('有限保持请求已受理：' + (response.message || ''), 'warn');
      await poll();
    } catch (error) {
      setMessage('保持被拒绝：' + describeError(error), 'error');
    }
  }

  async function doGravityFloat() {
    if (!allConfirmed() || !$('ack-gravity-float').checked) return;
    try {
      const response = await api('POST', '/api/gravity-float', {
        confirmations: Object.assign(confirmations(), { gravity_float: true }),
        float_settings: floatSettings()
      });
      setMessage('自由拖动已受理。托住等待3秒渐入，所设时间结束后承接重量。' + (response.message || ''), 'warn');
      await poll();
    } catch (error) {
      setMessage('自由拖动被拒绝：' + describeError(error), 'error');
    }
  }

  async function doGravityLock() {
    try {
      const response = await api('POST', '/api/gravity/lock', {});
      setMessage('锁定当前位置请求已受理；只有驱动实际模式与新鲜写回执确认后才显示已锁定。'
        + (response.message || ''), 'warn');
      await poll();
    } catch (error) {
      setMessage('锁定被拒绝：' + describeError(error), 'error');
      await poll();
    }
  }

  async function doGravityResume() {
    if (!$('ack-release-weight').checked) {
      setMessage('恢复自由拖动前请确认已承接重量，不会隐式恢复。', 'warn');
      return;
    }
    try {
      const response = await api('POST', '/api/gravity/float', {
        confirmations: Object.assign(confirmations(), { release_weight: true }),
      });
      setMessage('恢复自由拖动请求已受理；刚度平滑释放，仍需承接重量。' + (response.message || ''), 'warn');
      await poll();
    } catch (error) {
      setMessage('恢复自由拖动被拒绝：' + describeError(error), 'error');
      await poll();
    }
  }

  // ------------------------------------------------------------- 拖动示教
  function floatSettings() {
    return { kd: Number($('float-kd').value), seconds: Number($('float-seconds').value) };
  }

  function teachingAcks() {
    return Object.assign(confirmations(), { gravity_float: true, drag_teaching: true });
  }

  function teachingId() {
    return $('teaching-select').value || '';
  }

  async function doTeachingStart() {
    if (!$('ack-drag-teaching').checked) {
      setMessage('开始记录前请勾选拖动示教确认（托住机械臂、承接重量）。', 'warn');
      return;
    }
    if (!allConfirmed()) {
      setMessage('开始记录需要三项安全确认（服务端强制校验）。', 'warn');
      return;
    }
    try {
      const response = await api('POST', '/api/teaching/record/start', {
        confirmations: teachingAcks(),
        float_settings: floatSettings(),
      });
      S.ticket = null;
      S.teaching.ticket = null;
      setMessage('拖动示教已受理：托住等待渐入；记录只跟随有效窗口，停止走既有停止流程。'
        + (response.message || ''), 'warn');
      await poll();
    } catch (error) {
      setMessage('开始记录被拒绝：' + describeError(error), 'error');
    }
  }

  async function doTeachingStop() {
    try {
      await api('POST', '/api/teaching/record/stop', {});
      setMessage('停止记录已请求（走既有全轴停止，先停止后落盘）。', 'warn');
      await poll();
    } catch (error) {
      setMessage('停止记录被拒绝：' + describeError(error), 'error');
    }
  }

  async function doTeachingSave() {
    const teaching = (S.data && S.data.teaching) || {};
    const last = teaching.last || {};
    if (!last.recording_id) {
      setMessage('当前没有可保存的记录：请先完成一次拖动记录。', 'warn');
      return;
    }
    if (last.complete !== true) {
      setMessage('最近一次记录不完整，不能命名保存：' + ((last.reasons || []).join('；') || '未知原因'), 'error');
      return;
    }
    try {
      const response = await api('POST', '/api/teaching/save', {
        recording_id: last.recording_id, name: $('teaching-name').value,
      });
      setMessage('拖动示教已保存：' + response.teaching.name + '（'
        + response.teaching.frames + ' 帧 / ' + view(response.teaching.duration_s, 2) + ' 秒）', 'ok');
      await loadTeachingList();
      await loadSavedList();
      await poll();
    } catch (error) {
      setMessage('保存被拒绝：' + describeError(error), 'error');
    }
  }

  async function loadTeachingList() {
    try {
      const response = await api('GET', '/api/teachings');
      const select = $('teaching-select');
      const previous = select.value;
      select.textContent = '';
      const none = el('option', null, response.teachings.length ? '（选择已保存示教）' : '（暂无已保存示教）');
      none.value = '';
      select.appendChild(none);
      response.teachings.forEach((item) => {
        const option = el('option', null,
          item.name + ' · ' + (item.frames || 0) + '帧 · ' + view(item.duration_s, 2) + 's');
        option.value = item.id;
        select.appendChild(option);
      });
      if (previous) select.value = previous;
      if (!select.value && S.teaching.selected) {
        select.value = response.teachings.some((item) => item.id === S.teaching.selected.id)
          ? S.teaching.selected.id : '';
      }
    } catch (error) {
      setMessage('拖动示教列表读取失败：' + describeError(error), 'error');
    }
  }

  async function loadTeaching(id) {
    try {
      const response = await api('GET', '/api/teachings/' + encodeURIComponent(id));
      S.teaching.selected = response;
      S.teaching.ticket = null;
      setMessage('已载入示教：' + response.name + '（' + response.teaching.frames + ' 帧 / '
        + view(response.teaching.duration_s, 2) + ' 秒）。请预览回放，执行前再次确认。', 'ok');
      await poll();
    } catch (error) {
      setMessage('载入示教被拒绝：' + describeError(error), 'error');
    }
  }

  async function doTeachingLoad() {
    const id = teachingId();
    if (!id) {
      setMessage('请选择一个已保存的拖动示教。', 'warn');
      return;
    }
    await loadTeaching(id);
  }

  async function doTeachingPreview() {
    const id = teachingId();
    if (!id) {
      setMessage('请先载入一个已保存的拖动示教。', 'warn');
      return;
    }
    try {
      const response = await api('POST', '/api/teaching/preview', {
        teaching_id: id, speed_factor: Number($('teaching-speed').value),
      });
      S.teaching.ticket = response.ticket.id;
      S.ticket = null;
      S.teaching.preview = response.preview;
      setMessage('回放预览有效：接近 ' + view(response.preview.approach_s, 2) + ' 秒，总 '
        + view(response.preview.retimed_duration_s, 2) + ' 秒，峰值 '
        + view(response.preview.peak_speed_rad_s, 3) + ' rad/s；票根 '
        + response.ticket.id.slice(0, 8) + '…（未使能）', 'ok');
      await poll();
    } catch (error) {
      setMessage('回放预览被拒绝：' + describeError(error), 'error');
      await poll();
    }
  }

  async function doTeachingExecute() {
    if (!S.teaching.ticket) {
      setMessage('没有有效的回放预览票根：请先预览回放。', 'warn');
      return;
    }
    if (!allConfirmed()) {
      setMessage('执行回放需要三项安全确认（服务端强制校验）。', 'warn');
      return;
    }
    try {
      const response = await api('POST', '/api/teaching/execute', {
        ticket: S.teaching.ticket, confirmations: confirmations(),
      });
      S.teaching.ticket = null;
      setMessage('回放请求已受理（不是已完成）：正常位置控制，结果见“操作结果”。', 'warn');
      await poll();
    } catch (error) {
      setMessage('回放被拒绝：' + describeError(error), 'error');
      await poll();
    }
  }


  // ------------------------------------------------------------ 静态采样
  async function doCapturePreview() {
    try {
      const response = await api('POST', '/api/capture/preview', {});
      S.capture.ticket = response.ticket.id;
      S.capture.previewSignature = null;
      setMessage('静态采样预览已生成：起点 ° ' + (response.preview.start_deg || []).join(' / ')
        + '；α=' + view(response.preview.alpha, 3) + '（'
        + (response.preview.baseline && response.preview.baseline.satisfied ? '基线满足' : '仅诊断')
        + '）；票根 ' + response.ticket.id.slice(0, 8) + '…（未使能）', 'ok');
      await poll();
    } catch (error) {
      setMessage('静态采样预览被拒绝：' + describeError(error), 'error');
      await poll();
    }
  }

  async function doCaptureStart() {
    if (!S.capture.ticket) {
      setMessage('没有有效的静态采样预览票根：请先预览当前姿态。', 'warn');
      return;
    }
    if (!allConfirmed() || !$('ack-capture-start').checked) {
      setMessage('开始位置保持需要三项安全确认与位置保持确认（服务端强制）。', 'warn');
      return;
    }
    try {
      const response = await api('POST', '/api/capture/start', {
        ticket: S.capture.ticket,
        confirmations: Object.assign(confirmations(), { supported_static_capture: true }),
      });
      S.capture.ticket = null;
      setMessage('位置保持请求已受理：等待“保持已就绪”后托稳并缓慢撤手。', 'warn');
      await poll();
    } catch (error) {
      setMessage('位置保持被拒绝：' + describeError(error), 'error');
      await poll();
    }
  }

  async function doCaptureAck() {
    const capture = (S.data && S.data.supported_capture) || {};
    const current = capture.current || {};
    if (!current.id) {
      setMessage('没有进行中的静态采样位置保持。', 'warn');
      return;
    }
    if (!$('ack-operator-disengaged').checked) {
      setMessage('请先确认已缓慢撤手且防坠支撑不承重。', 'warn');
      return;
    }
    try {
      const response = await api('POST', '/api/capture/ack', {
        operation_id: current.id,
        confirmations: { operator_disengaged: true },
      });
      $('ack-operator-disengaged').checked = false;
      setMessage('撤手确认已记录：客户端正在采集静态窗口，随后停止并保存。', 'warn');
      await poll();
    } catch (error) {
      setMessage('撤手确认被拒绝：' + describeError(error), 'error');
      await poll();
    }
  }

  function renderCapture(data) {
    const capture = data.supported_capture || {};
    const current = capture.current || {};
    const caps = capture.capabilities || {};
    const baseline = capture.baseline || {};
    const cap = (name) => caps[name] || { ok: false, message: '状态未知' };
    renderFlags($('capture-flags'), [
      ['流程阶段', capture.phase || 'idle'],
      ['控制阶段', (data.ros && data.ros.stage_label) || '—'],
      ['驱动 α 基线', baseline.alpha === null || baseline.alpha === undefined
        ? '未知' : view(baseline.alpha, 3)],
      ['拟合基线', baseline.satisfied ? '满足（α=0）' : '不满足：样本仅诊断',
        baseline.satisfied ? 'cell-ok' : 'cell-bad'],
      ['驱动模式', (current.state && current.state.mode) || '—'],
      ['位置保持就绪', current.id ? (current.ready ? '是' : '否') : '—',
        current.id && current.ready ? 'cell-ok' : ''],
      ['命令 ID / boot', current.id
        ? (viewInt(current.command_id) + ' / ' + String(current.boot_id || '—').slice(0, 18)) : '—'],
      ['撤手确认', current.id ? (current.ack_sent
        ? '已发送 · ' + String(current.ack_nonce || '').slice(0, 8)
        : '未发送') : '—', current.ack_sent ? 'cell-ok' : ''],
      ['保持目标 °', current.id && current.target_deg
        ? current.target_deg.map((value) => view(value, 2)).join(' / ') : '—'],
    ]);
    const notes = [];
    if (baseline.note) notes.push(baseline.note);
    if (current.id && !current.ack_sent && (capture.readiness_reasons || []).length) {
      notes.push('等待保持就绪：' + capture.readiness_reasons.join('；'));
    }
    if (current.id && current.ready && !current.ack_sent) {
      notes.push('位置保持已就绪：托稳后缓慢撤手，防坠支撑不承重后再确认采样。');
    }
    if (current.ack_sent) notes.push('撤手确认已发送：客户端正在采集静态窗口，随后停止并保存。');
    $('capture-reason').textContent = notes.join(' ');

    const preview = capture.preview;
    const previewBox = $('capture-preview');
    if (!preview) {
      previewBox.classList.add('hidden');
    } else {
      previewBox.classList.remove('hidden');
      const validity = preview.validity || { valid: false, reasons: ['预览不可用'], drift_deg: null };
      const timing = preview.timing || {};
      renderFlags($('capture-preview-meta'), [
        ['预览票根', data.preview && data.preview.id ? data.preview.id.slice(0, 12) + '…' : '—'],
        ['校验状态', validity.valid ? '有效' : '已失效', validity.valid ? 'cell-ok' : 'cell-bad'],
        ['起点 °', (preview.start_deg || []).map((value) => view(value, 2)).join(' / ')],
        ['运行身份 boot', String((preview.identity || {}).boot_id || '—').slice(0, 24)],
        ['身份 SHA256', String((preview.identity || {}).identity_sha256 || '—').slice(0, 16) + '…'],
        ['Kp / Kd', view((preview.gains || {}).kp, 0) + ' / ' + view((preview.gains || {}).kd, 0)],
        ['α 基线', view(preview.alpha, 3) + '（'
          + ((preview.baseline || {}).satisfied ? '满足' : '仅诊断') + '）'],
        ['撤手等待上限', view(timing.ack_timeout_s, 0) + ' s'],
        ['静态窗口', view(timing.capture_seconds, 1) + ' s / ' + viewInt(timing.capture_samples) + ' 帧'],
        ['会话预算估计', view(timing.session_estimate_s, 1) + ' s'],
        ['起点漂移', validity.drift_deg === null ? '—' : view(validity.drift_deg, 2) + ' °'],
      ]);
      const warnings = $('capture-warnings');
      warnings.textContent = '';
      (preview.warnings || []).forEach((warning) => warnings.appendChild(el('li', null, warning)));
      if (!validity.valid) {
        (validity.reasons || []).forEach((reason) => warnings.appendChild(el('li', 'cell-bad', reason)));
      }
    }

    $('cap-capture').textContent = '预览：' + (cap('preview').ok ? '可用' : cap('preview').message)
      + '；开始：' + (cap('start').ok ? '可用' : cap('start').message)
      + '；撤手确认：' + (cap('ack').ok ? '可用' : cap('ack').message);

    const resultBox = $('capture-result');
    const last = capture.last;
    const result = last && last.result ? last.result : null;
    const sample = result && result.supported_capture ? result.supported_capture : null;
    if (!sample) {
      resultBox.classList.add('hidden');
      return;
    }
    resultBox.classList.remove('hidden');
    const window = sample.static_window || {};
    const residual = sample.residual_nm || [];
    const model = sample.model_gravity_nm || [];
    const feedback = sample.feedback_nm || [];
    renderFlags($('capture-result-meta'), [
      ['采样结果', sample.sample_valid ? '合格' : '不合格',
        sample.sample_valid ? 'cell-ok' : 'cell-bad'],
      ['拟合资格', sample.fit_eligible ? '可进入静态拟合' : '仅诊断',
        sample.fit_eligible ? 'cell-ok' : 'cell-warn'],
      ['α', view(sample.alpha, 3)],
      ['实测角度 °', (sample.positions_deg || []).map((value) => view(value, 3)).join(' / ')],
      ['静态窗口', viewInt(window.samples) + ' 帧 / ' + view(window.span_s, 2) + ' s / 间隙 '
        + view(window.max_gap_s, 3) + ' s'],
      ['窗口速度/位置变化', view(window.max_speed_rad_s, 4) + ' rad/s / '
        + view(window.max_position_delta_rad, 4) + ' rad'],
      ['反馈力矩 N·m', feedback.map((value) => view(value, 3)).join(' / ')],
      ['模型重力矩 N·m', model.map((value) => view(value, 3)).join(' / ')],
      ['只读残差 N·m', residual.map((value) => view(value, 3)).join(' / ')],
      ['原始文件', sample.raw_file || '—'],
      ['停止确认', result.stop_observations_confirmed ? '已确认新失能' : '未确认',
        result.stop_observations_confirmed ? 'cell-ok' : 'cell-bad'],
    ]);
    const resultNotes = [];
    if (!sample.sample_valid && (sample.invalid_reasons || []).length) {
      resultNotes.push('不合格原因：' + sample.invalid_reasons.join('；'));
    }
    if (sample.exclusion_reason) resultNotes.push('拟合排除：' + sample.exclusion_reason);
    if ((sample.baseline || {}).note) resultNotes.push(sample.baseline.note);
    resultNotes.push('残差为“反馈力矩估计 − 候选模型预测”，只读展示，不写参数或质量档案。');
    $('capture-result-note').textContent = resultNotes.join(' ');
  }

  // ------------------------------------------------------------ 单轴标定流程
  const AXIS_CAL_PHASES = ['zero', 'positive', 'lower', 'upper'];
  const AXIS_CAL_PHASE_LABELS = { zero: '零位读数', positive: '正方向移动',
    lower: '下限端点', upper: '上限端点' };

  function axisCalConfirmations() {
    return {
      axis_calibration_supported: $('ack-axis-cal-supported').checked === true,
      axis_calibration_clearance: $('ack-axis-cal-clearance').checked === true,
    };
  }

  function axisCalAcksOk() {
    const acks = axisCalConfirmations();
    return acks.axis_calibration_supported && acks.axis_calibration_clearance;
  }

  async function doAxisCalStart() {
    if (!axisCalAcksOk()) {
      setMessage('开始轴标定会话前请确认支撑与净空（服务端强制校验）。', 'warn');
      return;
    }
    try {
      const response = await api('POST', '/api/calibration/axis/start', {
        joint: $('axis-cal-joint').value || '',
        confirmations: axisCalConfirmations(),
      });
      setMessage('轴标定会话已开始（未使能、未运动）：' + (response.message || ''), 'ok');
      await poll();
    } catch (error) {
      setMessage('开始轴标定会话被拒绝：' + describeError(error), 'error');
      await poll();
    }
  }

  async function doAxisCalCapture(phase) {
    const display = ((S.data || {}).axis_calibration) || {};
    if (!display.session_id) {
      setMessage('没有进行中的轴标定会话：请先选择轴并开始会话。', 'warn');
      return;
    }
    if (!axisCalAcksOk()) {
      setMessage('采集前请确认支撑与净空（服务端强制校验）。', 'warn');
      return;
    }
    try {
      const response = await api('POST', '/api/calibration/axis/capture', {
        session_id: display.session_id, phase: phase, confirmations: axisCalConfirmations(),
      });
      setMessage('已采集' + (AXIS_CAL_PHASE_LABELS[phase] || phase) + '：'
        + (response.message || ''), 'ok');
      await poll();
    } catch (error) {
      setMessage('采集被拒绝：' + describeError(error), 'error');
      await poll();
    }
  }

  async function doAxisCalAbort() {
    try {
      const response = await api('POST', '/api/calibration/axis/abort', {
        reason: '操作员作废轴标定序列',
      });
      setMessage('轴标定序列已作废：' + (response.message || ''), 'warn');
      await poll();
    } catch (error) {
      setMessage('作废轴标定序列失败：' + describeError(error), 'error');
    }
  }

  function updateAxisCalButtons(data) {
    const display = (data && data.axis_calibration) || {};
    const caps = display.capabilities || {};
    const capOk = (name) => !!(caps[name] && caps[name].ok);
    const connected = S.connected;
    const busy = !!(data && data.operation && data.operation.running);
    const ackOk = axisCalAcksOk();
    const active = capOk('capture');
    $('btn-axis-cal-start').disabled = !connected || busy || !ackOk || !capOk('start');
    $('btn-axis-cal-abort').disabled = !connected || !capOk('abort');
    ['zero', 'positive', 'lower', 'upper'].forEach((phase) => {
      $('btn-axis-cal-' + phase).disabled = !connected || busy || !ackOk
        || !active || display.next_phase !== phase;
    });
    $('axis-cal-joint').disabled = !connected || active;
  }

  function renderAxisCalibration(data) {
    const display = data.axis_calibration || {};
    const joints = display.joints || [];
    const select = $('axis-cal-joint');
    if (select.options.length !== joints.length) {
      const previous = select.value;
      select.textContent = '';
      joints.forEach((joint) => {
        const option = el('option', null, joint.label || '—');
        option.value = joint.name;
        select.appendChild(option);
      });
      if (previous) select.value = previous;
      if (!select.value && display.selected_joint) select.value = display.selected_joint;
    }
    if (display.session_id && display.selected_joint) select.value = display.selected_joint;
    const caps = display.capabilities || {};
    const cap = (name) => caps[name] || { ok: false, message: '状态未知' };
    const direction = (display.candidate && display.candidate.direction) || null;
    const limits = (display.candidate && display.candidate.limits) || null;
    const sessionIdentity = display.session_identity || {};
    const live = display.live || {};
    renderFlags($('axis-cal-flags'), [
      ['流程阶段', display.phase_label || '—'],
      ['标定轴', display.selected_label || '（未开始）'],
      ['会话 ID', display.session_id ? display.session_id.slice(0, 12) + '…' : '—'],
      ['本会话 boot / 身份', sessionIdentity.boot_id
        ? String(sessionIdentity.boot_id).slice(0, 18) + '… / '
          + String(sessionIdentity.identity_sha256 || '').slice(0, 10) + '…' : '—'],
      ['mapping SHA256', String(sessionIdentity.mapping_sha256 || '').slice(0, 12) || '—'],
      ['model SHA256', String(sessionIdentity.model_sha256 || '').slice(0, 12) || '—'],
      ['全部轴未使能', live.disarmed ? '是' : '否', live.disarmed ? 'cell-ok' : 'cell-bad'],
      ['驱动运行身份', (display.identity || {}).fresh ? '就绪' : '未就绪/已过期',
        (display.identity || {}).fresh ? 'cell-ok' : 'cell-bad'],
      ['仍需采集', (display.pending || []).length ? display.pending.join(' / ')
        : (display.phase === 'complete' ? '无（仅候选记录）' : '—')],
    ]);
    const captures = display.captures || {};
    renderFlags($('axis-cal-captures'), [
      ['零位读数 °', finite(captures.zero_deg) ? view(captures.zero_deg, 3) : '—'],
      ['正方向读数 °', finite(captures.positive_deg) ? view(captures.positive_deg, 3) : '—'],
      ['正方向增量 °', finite(captures.positive_delta_deg) ? view(captures.positive_delta_deg, 3) : '—'],
      ['候选符号', direction && finite(direction.candidate_sign)
        ? (direction.candidate_sign > 0 ? '+1' : '-1') + '（候选，非正式）' : '—',
        direction ? 'cell-warn' : ''],
      ['下限端点 °', finite(captures.lower_deg) ? view(captures.lower_deg, 3) : '—'],
      ['上限端点 °', finite(captures.upper_deg) ? view(captures.upper_deg, 3) : '—'],
      ['候选区间 °', limits && finite(limits.lower_rad) && finite(limits.upper_rad)
        ? view(limits.lower_rad * 180 / Math.PI, 3) + ' ～ '
          + view(limits.upper_rad * 180 / Math.PI, 3) + '（候选，非正式）' : '—',
        limits ? 'cell-warn' : ''],
      ['正式记录', '未生成（不写零位 / mapping / URDF）', 'cell-warn'],
    ]);
    const notes = [];
    if ((display.reasons || []).length) notes.push('作废原因：' + display.reasons.join('；'));
    if (!display.session_id && (display.start_reasons || []).length) {
      notes.push('开始前检查：' + display.start_reasons.join('；'));
    }
    $('axis-cal-reason').textContent = notes.join(' ');
    $('cap-axis-cal').textContent = '开始：' + (cap('start').ok ? '可用' : cap('start').message)
      + '；采集：' + (cap('capture').ok ? '按顺序可用' : cap('capture').message)
      + '；作废：' + (cap('abort').ok ? '可用' : cap('abort').message);
    updateAxisCalButtons(data);
  }

  const TEACHING_PHASES = {
    idle: '未开始', starting: '启动中', ramp_in: '渐入中（尚未记录）', recording: '有效记录中',
    finalizing: '记录结束，正在落盘', ready: '记录完整，待命名保存', saved: '已保存',
    incomplete: '记录不完整（不能回放）', closed: '记录窗口已结束',
  };

  function renderTeaching(data) {
    const teaching = data.teaching || {};
    const last = teaching.last || {};
    const recording = teaching.recording || {};
    const phase = teaching.phase || 'idle';
    const caps = teaching.capabilities || {};
    const cap = (name) => caps[name] || { ok: false, message: '状态未知' };
    if (last.recording_id && last.recording_id !== S.teaching.lastId) {
      S.teaching.lastId = last.recording_id;
      loadTeachingList();
    }
    const rows = [
      ['记录阶段', TEACHING_PHASES[phase] || phase],
      ['驱动自由拖动', teaching.available ? '已开放' : '未开放', teaching.available ? 'cell-ok' : 'cell-bad'],
      ['驱动控制模式', (teaching.driver && teaching.driver.gravity_float) ? '自由拖动 Kp=0 / Kd=' + view(teaching.driver.float_kd, 1) : '位置保持 / 轨迹'],
      ['驱动已接收持续时间', teaching.driver && teaching.driver.float_configurable ? view(teaching.driver.float_seconds, 1) + ' s' : '—'],
      ['运行身份', teaching.identity && teaching.identity.fresh
        ? '已就绪 ' + String(teaching.identity.boot_id || '').slice(0, 18) : '未就绪 / 已过期',
        teaching.identity && teaching.identity.fresh ? 'cell-ok' : 'cell-bad'],
      ['默认速度比例', view(teaching.default_speed_factor, 2) + '（可调 ' + view(teaching.speed_factor_min, 2)
        + '～' + view(teaching.speed_factor_max, 2) + '）'],
      ['回放峰值硬上限', view(teaching.speed_limit_rad_s, 2) + ' rad/s'],
    ];
    if (recording && Object.keys(recording).length) {
      rows.push(['当前记录帧数', viewInt(recording.frames) + ' 帧（原始 ' + viewInt(recording.raw_frames) + '）']);
      rows.push(['当前记录时长', view(recording.duration_s, 2) + ' s']);
      rows.push(['当前命令 ID / boot', viewInt(recording.command_id) + ' / '
        + String(recording.boot_id || '—').slice(0, 18)]);
    }
    if (last.recording_id) {
      const quality = last.quality || {};
      rows.push(['最近记录结果', last.complete ? '完整' : '不完整',
        last.complete ? 'cell-ok' : 'cell-bad']);
      rows.push(['最近记录帧数/时长', viewInt(quality.frames || last.frames) + ' 帧 / '
        + view(quality.duration_s, 2) + ' s']);
      rows.push(['最近记录最大间隙/年龄', view(quality.max_gap_s, 4) + ' s / '
        + view(quality.max_age_s, 4) + ' s']);
      rows.push(['最近记录命令 ID', viewInt(last.command_id)]);
      if (!last.complete && (last.reasons || []).length) {
        rows.push(['不完整原因', (last.reasons || []).slice(0, 3).join('；')]);
      }
      if (last.saved && last.teaching) {
        rows.push(['已保存为', last.teaching.name + '（' + String(last.teaching.id || '').slice(0, 8) + '…）']);
      }
    }
    renderFlags($('teaching-flags'), rows);
    const notes = [];
    if (teaching.last_error) notes.push('记录错误：' + teaching.last_error);
    if (phase === 'ramp_in') notes.push('等待渐入完成：此时拖动不会进入记录。');
    if (phase === 'recording') notes.push('有效记录中：请保持自由拖动，停止记录会走既有全轴停止。');
    if (phase === 'incomplete' && (last.reasons || []).length) {
      notes.push('不完整记录不能回放：' + last.reasons.slice(0, 3).join('；'));
    }
    $('teaching-reason').textContent = notes.join(' ');
    const startCap = cap('start');
    const previewCap = cap('preview');
    const executeCap = cap('execute');
    $('cap-teaching').textContent = '记录：' + (startCap.ok ? '可开始' : startCap.message)
      + '；回放预览：' + (previewCap.ok ? '可用' : previewCap.message)
      + '；执行：' + (executeCap.ok ? '可用' : executeCap.message);
    renderTeachingPreview(data);
  }

  function renderTeachingPreview(data) {
    const wrapper = $('teaching-preview');
    const ticket = data.preview;
    const payload = ticket && ticket.preview;
    if (!ticket || !payload || payload.kind !== 'teaching') {
      wrapper.classList.add('hidden');
      S.teaching.previewSignature = null;
      return;
    }
    wrapper.classList.remove('hidden');
    const validity = ticket.validity || { valid: false, reasons: ['预览不可用'], drift_deg: null };
    const signature = ticket.id + '|' + ticket.used + '|' + (validity.reasons || []).join(',');
    if (signature !== S.teaching.previewSignature) {
      S.teaching.previewSignature = signature;
      renderFlags($('teaching-preview-meta'), [
        ['票根', ticket.id.slice(0, 12) + '…'],
        ['校验状态', validity.valid ? '有效' : '已失效', validity.valid ? 'cell-ok' : 'cell-bad'],
        ['剩余有效期', view(ticket.remaining_s, 1) + ' s'],
        ['示教名称', (payload.teaching && payload.teaching.name) || '—'],
        ['速度比例', view(payload.speed_factor, 2) + '（默认 ' + view(payload.speed_factor_default, 2) + '）'],
        ['接近段', view(payload.approach_s, 2) + ' s，峰值 ' + view(payload.peak_approach_speed_rad_s, 2) + ' rad/s'],
        ['原记录时长', view(payload.original_duration_s, 2) + ' s'],
        ['重定时总时长', view(payload.retimed_duration_s, 2) + ' s（接近 + 记录）'],
        ['峰值回放速度', view(payload.peak_speed_rad_s, 3) + ' rad/s（上限 ' + view(payload.speed_limit_rad_s, 2) + '）'],
        ['插值弯曲界', view(payload.interp_error_bound_deg, 3) + ' °（相邻采样线性插值）'],
        ['起点漂移', validity.drift_deg === null ? '—' : view(validity.drift_deg, 2) + ' °'],
        ['接近目标 °', (payload.target_start_deg || []).map((v) => view(v, 2)).join(' / ')],
      ]);
      const list = $('teaching-warnings');
      list.textContent = '';
      (payload.warnings || []).forEach((warning) => list.appendChild(el('li', null, warning)));
      if (!validity.valid) {
        (validity.reasons || []).forEach((reason) => list.appendChild(el('li', 'cell-bad', reason)));
      }
    }
  }

  async function doConnectionCheck() {
    try {
      const value = await api('GET', '/api/connection');
      setMessage('连接检查：串口' + (value.port_present ? '存在' : '未发现')
        + '；驱动进程 ' + (value.launch_pids || []).length
        + '；五轴反馈' + (value.feedback_valid_all ? '有效' : '无效或失联')
        + '。' + ((value.state || {}).message || ''), value.feedback_valid_all ? 'ok' : 'warn');
    } catch (error) { setMessage('连接检查失败：' + describeError(error), 'error'); }
  }

  async function doReconnect() {
    try {
      $('btn-reconnect').disabled = true;
      const response = await api('POST', '/api/connection/reconnect', {
        ack_power_cycle: $('ack-power-cycle').checked,
      });
      $('ack-power-cycle').checked = false;
      S.ticket = null;
      markDirty();
      setMessage(response.message || '正在恢复通信，不会使能电机', 'warn');
      await poll();
    } catch (error) { setMessage('重连未完成：' + describeError(error), 'error'); }
    finally { if (!(S.data && S.data.connection && S.data.connection.phase === 'connecting')) $('btn-reconnect').disabled = false; }
  }

  async function doReset() {
    try {
      const response = await api('POST', '/api/reset', {});
      setMessage('软件复位请求已排队（不会自动使能）：' + (response.message || ''), 'ok');
      await poll();
    } catch (error) {
      setMessage('复位被拒绝：' + describeError(error), 'error');
    }
  }

  async function doStop() {
    setMessage('正在请求全轴停止…', 'warn');
    try {
      const response = await api('POST', '/api/stop', { reason: '操作员点击全轴停止' });
      setMessage('停止已请求（不是物理急停）：等待新失能确认。不要重复使能。', 'warn');
      await poll();
      return response;
    } catch (error) {
      setMessage('停止请求失败：' + describeError(error) + '。请立即现场切断电机电源。', 'error');
    }
  }

  async function savePlan() {
    const parsed = planFromEditor();
    if (!parsed.ok) {
      setMessage(parsed.message, 'error');
      return;
    }
    try {
      const response = await api('POST', '/api/plans/save', {
        name: $('plan-name').value, document: parsed.document,
      });
      setMessage('计划已保存到存档：' + response.plan.id + '（' + response.plan.name + '）', 'ok');
      await loadSavedList();
    } catch (error) {
      setMessage('保存被拒绝：' + describeError(error), 'error');
    }
  }

  async function loadSavedList() {
    try {
      const response = await api('GET', '/api/plans');
      const select = $('saved-select');
      select.textContent = '';
      const none = el('option', null, '（无存档）');
      none.value = '';
      select.appendChild(none);
      response.plans.forEach((plan) => {
        const label = plan.kind === 'teaching'
          ? '[示教] ' + plan.name + ' · ' + (plan.axes === null || plan.axes === undefined ? '?' : plan.axes)
            + '轴 · ' + (plan.frames || 0) + '帧 · ' + view(plan.duration_s, 2) + 's'
          : plan.name + ' · ' + (plan.axes === null || plan.axes === undefined ? '?' : plan.axes) + '轴 · '
            + plan.waypoints + '点 · ' + (plan.schema || '旧格式') + ' · ' + dateOf(plan.saved_at);
        const option = el('option', null, label);
        option.value = plan.id;
        select.appendChild(option);
      });
    } catch (error) {
      setMessage('存档列表读取失败：' + describeError(error), 'error');
    }
  }

  async function loadSaved() {
    const id = $('saved-select').value;
    if (!id) {
      setMessage('请选择一个存档。', 'warn');
      return;
    }
    try {
      const response = await api('GET', '/api/plans/' + encodeURIComponent(id));
      if (response.kind === 'teaching') {
        S.teaching.selected = response;
        S.teaching.ticket = null;
        if ($('teaching-select').querySelector('option[value="' + id + '"]')) {
          $('teaching-select').value = id;
        }
        setMessage('已从现有存档列表载入拖动示教：' + response.name + '。请预览回放后再执行。', 'ok');
        await poll();
        return;
      }
      applyPlan(response.document, response.name);
    } catch (error) {
      setMessage('载入被拒绝：' + describeError(error), 'error');
    }
  }

  function exportJson() {
    const parsed = planFromEditor();
    if (!parsed.ok) {
      setMessage(parsed.message, 'error');
      return;
    }
    const text = JSON.stringify(parsed.document, null, 2);
    const blob = new Blob([text], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const anchor = el('a');
    const stamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
    anchor.href = url;
    anchor.download = 'five_axis_plan_' + stamp + '.json';
    document.body.appendChild(anchor);
    anchor.click();
    anchor.remove();
    setTimeout(() => URL.revokeObjectURL(url), 2000);
    setMessage('已导出当前编辑计划（数据文件，不等于预览或执行）。', 'ok');
  }

  function importJson(file) {
    if (!file) return;
    if (file.size > 65536) {
      setMessage('导入文件超过 64KB 限制。', 'error');
      return;
    }
    const reader = new FileReader();
    reader.onload = () => {
      try {
        const plan = JSON.parse(String(reader.result));
        applyPlan(plan, file.name);
      } catch (error) {
        setMessage('导入失败（只当数据处理，不执行其中指令）：' + error.message, 'error');
      }
    };
    reader.onerror = () => setMessage('读取导入文件失败。', 'error');
    reader.readAsText(file, 'utf-8');
  }

  // ------------------------------------------------------------------ render
  function buildAxes() {
    const body = $('axes-body');
    for (let index = 0; index < AXIS_COUNT; index += 1) {
      const tr = el('tr');
      const cells = [];
      for (let column = 0; column < 11; column += 1) {
        const cell = el('td');
        tr.appendChild(cell);
        cells.push(cell);
      }
      cells[0].textContent = 'J' + (index + 1);
      body.appendChild(tr);
      S.axisCells.push(cells);
    }
  }

  function setCell(cell, text, cls) {
    if (cell.textContent !== text) cell.textContent = text;
    const value = cls || '';
    if (cell.className !== value) cell.className = value;
  }

  function setTab(name) {
    const main = document.querySelector('main');
    if (!main) return;
    main.dataset.tab = name;
    document.querySelectorAll('.tab').forEach((button) => {
      const active = button.dataset.tab === name;
      button.classList.toggle('active', active);
      button.setAttribute('aria-selected', active ? 'true' : 'false');
    });
  }

  function renderChips(data) {
    const ros = data.ros || {};
    const chipRos = $('chip-ros');
    const axes = Array.isArray(data.axes) ? data.axes : [];
    const staleAxes = axes.filter((row) => row.feedback_valid !== true
      || !finite(row.feedback_age_s) || row.feedback_age_s > 0.3);
    if (!ros.connected) {
      chipRos.textContent = '反馈：失联';
      chipRos.className = 'chip chip-bad';
    } else if (staleAxes.length) {
      chipRos.textContent = '反馈：' + staleAxes.map((row) => row.label || '?').join('/') + ' 陈旧';
      chipRos.className = 'chip chip-warn';
    } else if (finite(ros.dynamic_age_s) && ros.dynamic_age_s > 0.3) {
      chipRos.textContent = '反馈：延迟 ' + view(ros.dynamic_age_s) + 's';
      chipRos.className = 'chip chip-warn';
    } else {
      chipRos.textContent = '反馈：在线';
      chipRos.className = 'chip chip-ok';
    }
    const chipStage = $('chip-stage');
    const stage = ros.stage;
    chipStage.textContent = '阶段：' + (ros.stage_label || '—');
    chipStage.className = 'chip ' + (stage === 9 ? 'chip-bad' : stage === 0 ? 'chip-ok'
      : stage === 8 ? 'chip-warn' : stage === null || stage === undefined ? 'chip-wait' : 'chip-warn');
    const lease = data.lease || {};
    const chipLease = $('chip-lease');
    const running = !!(data.operation && data.operation.running);
    if (!lease.fresh) {
      chipLease.textContent = '心跳：过期 ' + view(lease.last_heartbeat_age_s) + 's';
      chipLease.className = 'chip ' + (running ? 'chip-bad' : 'chip-warn');
    } else {
      chipLease.textContent = '心跳：正常';
      chipLease.className = 'chip chip-ok';
    }
    const chipLatency = $('chip-latency');
    if (chipLatency) {
      if (!finite(S.pollMs)) {
        chipLatency.textContent = '面板：—';
        chipLatency.className = 'chip chip-wait';
      } else {
        chipLatency.textContent = '面板：' + view(S.pollMs, 0) + 'ms';
        chipLatency.className = 'chip ' + (S.pollMs > 1000 ? 'chip-bad'
          : S.pollMs > 300 ? 'chip-warn' : 'chip-ok');
      }
    }
  }

  function renderAxisSummary(data) {
    const host = $('axis-summary');
    if (!host) return;
    host.textContent = '';
    const rows = data.axes || [];
    rows.forEach((row) => {
      const ok = row.feedback_valid === true && finite(row.feedback_age_s);
      const qualification = typeof row.qualification === 'string' ? row.qualification.trim() : '';
      const pending = qualification.length > 0 && qualification !== '—';
      const card = el('div', 'axis-card ' + (pending ? 'axis-pending' : ok ? 'axis-ok' : 'axis-bad'));
      card.appendChild(el('div', 'axis-name', row.label || '—'));
      card.appendChild(el('div', 'axis-value', finite(row.actual_deg) ? view(row.actual_deg, 2) + '°' : '—'));
      const status = pending ? '运动待定' : ok ? '反馈正常' : (row.feedback_valid ? '反馈陈旧' : '无反馈');
      if (pending) card.title = qualification;
      card.appendChild(el('div', 'axis-status', status));
      card.appendChild(el('div', 'axis-age', finite(row.feedback_age_s) ? view(row.feedback_age_s, 3) + ' s' : ''));
      host.appendChild(card);
    });
    if (!rows.length) host.appendChild(el('p', 'note', '尚无五轴状态。'));
  }

  function renderBanner(data) {
    const banner = $('banner');
    const ros = data.ros || {};
    const unified = data.unified || {};
    const axisLabel = unified.axis_label || '五轴';
    const stop = data.stop || {};
    const messages = [];
    let level = '';
    if (data.server && data.server.closing) {
      messages.push('面板正在退出：在途客户端将被终止并请求停止。');
      level = 'bad';
    }
    if (ros.stage === 9) {
      messages.push('控制端 UNKNOWN：' + axisLabel + '新失能未确认。保持机械支撑并现场切断电机电源，不要重复使能。');
      level = 'bad';
    }
    if (stop.state === 'unknown') {
      messages.push('停止后未确认新失能：状态未知，保持支撑并现场切电。');
      level = 'bad';
    } else if (stop.state === 'requested') {
      messages.push('停止已请求，等待新的' + axisLabel + '失能反馈确认（不是物理急停）。');
      level = level || 'warn';
    }
    if (!ros.connected) {
      messages.push(axisLabel + '反馈失联：不能用 0 假填充，禁止预览/执行。');
      level = level || 'warn';
    }
    if (unified.preflight && unified.preflight.ok === false) {
      messages.push(axisLabel + '预检查拒绝全部运动：' + (unified.preflight.reasons || []).join('；'));
      level = level === 'bad' ? level : 'warn';
    }
    if (data.operation && data.operation.running) {
      messages.push('操作进行中：' + (data.operation.kind === 'hold' ? '有限保持'
        : data.operation.kind === 'gravity_float' ? '重力补偿自由拖动'
          : data.operation.kind === 'drag_teaching' ? '拖动示教记录'
            : '执行路点计划')
        + '，停止入口始终可用。');
      level = level || 'ok';
    }
    const gate = data.stop_gate || {};
    if (gate.locked) {
      messages.push('停止已受理：预览/执行/保持锁定，需手动复位并观测到新的' + axisLabel + '一致未使能。');
      level = level === 'bad' ? level : 'warn';
    }
    if (!messages.length) {
      banner.className = 'banner hidden';
      banner.textContent = '';
      return;
    }
    banner.className = 'banner' + (level ? ' ' + level : '');
    banner.textContent = messages.join(' ');
  }

  function renderAxes(data) {
    const axes = data.axes || [];
    for (let index = 0; index < AXIS_COUNT; index += 1) {
      const axis = axes[index] || {};
      const cells = S.axisCells[index];
      const valid = axis.feedback_valid === true;
      const actual = finite(axis.actual_deg) ? view(axis.actual_deg, 3) : (valid ? '—' : '—/失联');
      const fullQualification = axis.qualification && axis.qualification !== '—'
        ? axis.qualification : '—';
      const qualification = fullQualification === '合格' || fullQualification === '—'
        ? fullQualification : '待定';
      setCell(cells[1], qualification, qualification === '合格' ? 'cell-ok'
        : qualification === '—' ? 'cell-missing' : 'cell-bad');
      if (fullQualification !== qualification) cells[1].title = fullQualification;
      setCell(cells[2], actual, finite(axis.actual_deg) ? '' : 'cell-missing');
      setCell(cells[3], view(axis.target_deg, 3), finite(axis.target_deg) ? '' : 'cell-missing');
      setCell(cells[4], view(axis.error_deg, 3), finite(axis.error_deg) && Math.abs(axis.error_deg) > 0.5 ? 'cell-bad' : '');
      setCell(cells[5], view(axis.velocity_deg_s, 2), '');
      setCell(cells[6], view(axis.effort_nm, 3), '');
      setCell(cells[7], view(axis.driver_temperature_c, 1), '');
      setCell(cells[8], view(axis.motor_temperature_c, 1), '');
      setCell(cells[9], view(axis.feedback_age_s, 3), '');
      setCell(cells[10], valid ? '有效' : '失联', valid ? 'cell-ok' : 'cell-bad');
    }
  }

  function renderFlags(container, entries) {
    container.textContent = '';
    entries.forEach((entry) => {
      const row = el('div');
      row.appendChild(el('dt', null, entry[0]));
      row.appendChild(el('dd', entry[2] ? entry[2] : null, entry[1]));
      container.appendChild(row);
    });
  }

  function renderMotionFlags(data) {
    const ros = data.ros || {};
    const unified = data.unified || {};
    const j5 = unified.j5 || {};
    const j5Feedback = unified.j5_feedback || {};
    const preflight = unified.preflight || {};
    const gravityIdentity = unified.gravity_identity || {};
    const identity = unified.identity || {};
    const known = ros.stage !== null && ros.stage !== undefined;
    const j5Reasons = (j5.reasons || []).length ? j5.reasons.join('；') : '无';
    renderFlags($('motion-flags'), [
      ['控制阶段', ros.stage_label + (known ? '（' + ros.stage + '）' : '')],
      ['受控轴', (unified.joint_names || []).map((name) => name.replace('slaihand_joint', 'J')).join(' / ') || '—'],
      ['计划版本', unified.schema || '—'],
      ['运行身份', identity.fresh
        ? 'boot ' + String(identity.boot_id || '—').slice(0, 18) + ' · load ' + (identity.load_id || '—')
          + ' · ' + view(identity.age_s, 1) + ' s'
        : '未就绪/已过期：' + (identity.reason || '未收到'),
        identity.fresh ? 'cell-ok' : 'cell-bad'],
      ['五轴预检查', preflight.ok === true ? '通过' : '拒绝全部运动',
        preflight.ok === true ? 'cell-ok' : 'cell-bad'],
      ['J5 资格', j5.required === false ? '四轴模式不需要' : (j5.qualified === true ? '合格' : '待定/拒绝：' + j5Reasons),
        j5.qualified === true || j5.required === false ? 'cell-ok' : 'cell-bad'],
      ['J5 反馈时效', finite(j5Feedback.feedback_age_s) ? view(j5Feedback.feedback_age_s, 3) + ' s' : '—',
        finite(j5Feedback.feedback_age_s) ? '' : 'cell-missing'],
      ['重力/负载身份', (gravityIdentity.status || '—') + '（正式重力：'
        + (gravityIdentity.formal_gravity_allowed ? '允许' : '拒绝') + '）',
        gravityIdentity.formal_gravity_allowed ? 'cell-ok' : 'cell-bad'],
      ['五轴一致', known ? bool(ros.consistent) : '—'],
      ['稳定保持就绪', known ? bool(ros.hold_ready) : '—'],
      ['新失能已观察', known ? bool(ros.disabled_observed) : '—'],
      ['控制器已接管', known ? bool(ros.command_owner) : '—'],
      ['审计可用', known ? bool(ros.audit_ok) : '—'],
      ['后端命令 ID', viewInt(ros.command_id)],
      ['动态反馈时效', view(ros.dynamic_age_s, 3) + ' s'],
      ['关节反馈时效', view(ros.joint_age_s, 3) + ' s'],
      ['控制器状态时效', view(ros.controller_age_s, 3) + ' s'],
      ['轨迹时间倍率',view(ros.speed_scaling_factor,2)],
      ['目标来源', (data.axes && data.axes[0] && data.axes[0].target_source) || '—'],
      ['限位来源', ros.limits_deg ? '完整 URDF + J5 硬件映射' : '不可用'],
      ['停止门禁', (data.stop_gate && data.stop_gate.locked) ? '锁定（等待手动复位+新 DISARMED）' : '未锁定',
        (data.stop_gate && data.stop_gate.locked) ? 'cell-bad' : ''],
    ]);
  }

  function renderDiagnostics(data) {
    const diagnostics = data.diagnostics || {};
    const gains = diagnostics.gains || {};
    const gainsBody = $('gains-body');
    if (gainsBody) {
      gainsBody.textContent = '';
      const gainRows = gains.axes || [];
      gainRows.forEach((row) => {
        const tr = el('tr');
        const statusText = row.status || '—';
        const statusClass = statusText === 'VERIFIED' ? 'cell-ok'
          : (statusText === 'SCALAR_ONLY' || statusText === 'UNKNOWN') ? 'cell-missing' : 'cell-bad';
        const values = [
          row.label || '—',
          finite(row.kp_applied) ? view(row.kp_applied, 3)
            : (finite(row.kp_configured) ? '标量 ' + view(row.kp_configured, 3) : '—'),
          finite(row.kd_applied) ? view(row.kd_applied, 3)
            : (finite(row.kd_configured) ? '标量 ' + view(row.kd_configured, 3) : '—'),
          view(row.kp_encoded, 3),
          view(row.kd_encoded, 3),
          statusText,
          row.kp_source || row.kd_source || '—',
        ];
        values.forEach((text, index) => {
          tr.appendChild(el('td', index === 5 ? statusClass : '', text));
        });
        tr.title = (row.reasons || []).join('；');
        gainsBody.appendChild(tr);
      });
      if (!gainRows.length) {
        const tr = el('tr');
        const cell = el('td', 'cell-missing', '尚无逐轴 Kp/Kd 信息（等待驱动运行身份）');
        cell.colSpan = 7;
        tr.appendChild(cell);
        gainsBody.appendChild(tr);
      }
    }
    const gainsNote = $('gains-note');
    if (gainsNote) gainsNote.textContent = gains.display || (gains.reasons || []).join('；') || '—';

    const twoLayer = diagnostics.two_layer || {};
    const layerBody = $('fault-layers-body');
    if (layerBody) {
      layerBody.textContent = '';
      (twoLayer.layers || []).forEach((layer) => {
        const tr = el('tr');
        let errorText = '—';
        if (layer.layer === 'JTC') {
          errorText = '路径门限 ' + view(layer.tolerance_deg, 2) + '°'
            + (finite(layer.goal_tolerance_deg) ? ' / 目标 ' + view(layer.goal_tolerance_deg, 2) + '°' : '');
        } else if (finite(layer.error_deg) || finite(layer.limit_deg)) {
          errorText = view(layer.error_deg, 4) + ' / ' + view(layer.limit_deg, 2);
        }
        const stamp = layer.layer === 'JTC'
          ? (layer.source || '—')
          : (finite(layer.detected_at) ? view(layer.detected_at, 3) + ' s(steady)' : '—');
        const values = [layer.layer || '—', layer.role || '—', layer.axis_label || '—',
          errorText, stamp, layer.display || '—'];
        values.forEach((text, index) => {
          tr.appendChild(el('td', index === 0 && layer.available !== true ? 'cell-missing' : '', text));
        });
        layerBody.appendChild(tr);
      });
    }

    const first = diagnostics.first_fault || {};
    const thresholds = twoLayer.thresholds || {};
    const faultFlags = $('fault-flags');
    const laterEvents = twoLayer.later_events || [];
    if (faultFlags) {
      renderFlags(faultFlags, [
        ['驱动首因', first.available
          ? (first.code || '可用')
          : (twoLayer.first_cause_missing_reason || '未收到（UNKNOWN，不推断）'),
          first.available ? 'cell-bad' : 'cell-missing'],
        ['首因轴', first.available ? (first.axis_label || '全轴/非单轴') : '—'],
        ['误差 / 驱动门限', finite(first.error_deg)
          ? view(first.error_deg, 4) + '° / ' + (finite(first.limit_deg)
            ? view(first.limit_deg, 2) + '°' : '—') : '—',
          first.exceeded === true ? 'cell-bad' : ''],
        ['目标 / 实际', first.target_actual_available
          ? view(first.target_deg, 4) + '° / ' + view(first.actual_deg, 4) + '°' : '—'],
        ['编码 Kp / Kd / tau_ff', finite(first.encoded_kp)
          ? view(first.encoded_kp, 3) + ' / ' + view(first.encoded_kd, 3) + ' / '
            + view(first.encoded_tau_ff_nm, 3) + ' N·m' : '—'],
        ['反馈时效 / 有效', finite(first.feedback_age_s)
          ? view(first.feedback_age_s, 3) + ' s / ' + bool(first.feedback_valid) : '—'],
        ['command / cycle / boot', first.available
          ? [viewInt(first.command_id), viewInt(first.cycle_id),
            String(first.boot_id || '—').slice(0, 18)].join(' · ') : '—'],
        ['最后完整写入 / 距故障', finite(first.last_complete_write_at)
          ? view(first.last_complete_write_at, 3) + ' s / ' + view(first.last_write_age_s, 3) + ' s' : '—'],
        ['首因保留', twoLayer.first_cause_preserved
          ? '已冻结保留；后续停止/JTC 不替代首因'
          : (twoLayer.first_cause_missing_reason || '—'),
          twoLayer.first_cause_preserved ? 'cell-ok' : 'cell-bad'],
        ['后续事件（非首因）', laterEvents.length
          ? laterEvents.map((event) => (event.layer || '?') + '：'
            + (event.message || event.code || '')).join('；')
          : '无', laterEvents.length ? 'cell-bad' : 'cell-ok'],
      ]);
    }
    const estimate = diagnostics.request_estimate || {};
    const estimateNode = $('fault-estimate');
    if (estimateNode) {
      const valueText = finite(estimate.value_nm) ? '（' + view(estimate.value_nm, 3) + ' N·m）' : '';
      estimateNode.textContent = '请求估算：' + (estimate.label || '不可用') + valueText
        + (estimate.is_current_output ? '' : '；从不作为当前输出/实测力矩');
    }
    const thresholdFlags = $('fault-thresholds');
    if (thresholdFlags) {
      renderFlags(thresholdFlags, [
        ['驱动 FOLLOWING_ERROR 门限', finite(thresholds.driver_guard_deg)
          ? view(thresholds.driver_guard_deg, 2) + '°' : '—'],
        ['首因快照门限', finite(thresholds.driver_first_fault_limit_deg)
          ? view(thresholds.driver_first_fault_limit_deg, 2) + '°（'
            + (thresholds.driver_limit_source || '—') + '）' : '—'],
        ['JTC 路径门限', finite(thresholds.jtc_tolerance_deg)
          ? view(thresholds.jtc_tolerance_deg, 2) + '°' : '—'],
        ['JTC 目标门限', finite(thresholds.jtc_goal_tolerance_deg)
          ? view(thresholds.jtc_goal_tolerance_deg, 2) + '°' : '—'],
        ['两层门限', thresholds.matched === true ? '一致'
          : thresholds.matched === false
            ? '不一致：JTC 比驱动宽 ' + view(thresholds.mismatch_deg, 2) + '°' : '—',
          thresholds.matched === true ? 'cell-ok' : 'cell-bad'],
        ['候选恢复', (diagnostics.recommendation || {}).status || '—'],
      ]);
    }
    const recommendationNode = $('jtc-recommendation');
    if (recommendationNode) {
      const recommendation = diagnostics.recommendation || {};
      const changes = recommendation.changes || [];
      recommendationNode.textContent = changes.join('；')
        + (recommendation.note ? ' ' + recommendation.note : '');
    }
  }

  function renderMargin(data) {
    const margin = data.margin || {};
    const body = $('margin-body');
    if (body) {
      body.textContent = '';
      const rows = margin.axes || [];
      rows.forEach((row) => {
        const capability = row.capability || {};
        const demand = row.demand || {};
        const duration = capability.duration_kind === 'continuous' ? '持续'
          : capability.duration_kind === 'peak'
            ? '峰值 ' + view(capability.duration_s, 2) + ' s'
            : (capability.duration_kind || '—');
        const statusText = row.status_label || row.status || '—';
        const statusClass = row.status === 'PASS' ? 'cell-ok'
          : (row.status === 'UNKNOWN' || row.status === 'PRESCREEN_PASS') ? 'cell-missing'
            : 'cell-bad';
        const values = [
          row.label || '—',
          finite(capability.value_nm) ? view(capability.value_nm, 3) + ' N·m' : '—',
          [capability.source || '—', capability.unit || '单位?'].join(' / '),
          duration,
          capability.condition || '—',
          finite(demand.value_nm) ? view(demand.value_nm, 3) + ' N·m' : '—',
          [demand.source || '—', demand.measurement_type || '—'].join(' / '),
          finite(row.safety_factor) ? view(row.safety_factor, 3) : '—',
          statusText,
        ];
        const tr = el('tr');
        values.forEach((text, index) => {
          tr.appendChild(el('td', index === 8 ? statusClass : '', text));
        });
        tr.title = (row.reasons || []).join('；');
        body.appendChild(tr);
      });
      if (!rows.length) {
        const tr = el('tr');
        const cell = el('td', 'cell-missing', '尚无五轴裕量数据（UNKNOWN）');
        cell.colSpan = 9;
        tr.appendChild(cell);
        body.appendChild(tr);
      }
    }
    const flags = $('margin-flags');
    if (flags) {
      renderFlags(flags, [
        ['总体', margin.overall_label || margin.overall || '—',
          margin.overall === 'PASS' ? 'cell-ok' : margin.overall === 'FAIL'
            || margin.overall === 'NOT_QUALIFIED' ? 'cell-bad' : 'cell-missing'],
        ['数据状态', margin.data_status || 'UNKNOWN'],
        ['通过 / 未知 / 不足 / 不合格',
          (margin.pass_axes || 0) + ' / ' + (margin.unknown_axes || 0) + ' / '
          + (margin.fail_axes || 0) + ' / ' + (margin.not_qualified_axes || 0)],
        ['规则', margin.rule || 'allowed / demand >= 1.5'],
        ['能力来源', margin.source || '未提供（UNKNOWN）'],
        ['物理验收', margin.physical_acceptance === true ? '已声称' : '未声称（本表不是实测验收）',
          'cell-missing'],
      ]);
    }
    const gate = $('margin-gate');
    if (gate) {
      const qualification = margin.qualification || {};
      const reasons = qualification.reasons || [];
      gate.textContent = '调参/扩幅资格：' + (qualification.label || '未评估')
        + (reasons.length ? '；原因：' + reasons.slice(0, 6).join('；') : '');
      gate.className = 'note ' + (qualification.tuning_allowed === true ? 'ok' : '');
    }
    const disclaimer = $('margin-disclaimer');
    if (disclaimer) disclaimer.textContent = margin.disclaimer || '';
  }

  function renderPreview(data) {
    const preview = data.preview;
    const panel = $('preview-panel');
    const empty = $('preview-empty');
    if (!preview) {
      panel.classList.add('hidden');
      empty.classList.remove('hidden');
      empty.textContent = '尚未预览。';
      S.rendered.preview = null;
      return;
    }
    panel.classList.remove('hidden');
    empty.classList.add('hidden');
    const payload = preview.preview || {};
    if (payload.kind === 'teaching') {
      // Teaching tickets render in the drag-teaching card, not in the waypoint table.
      panel.classList.add('hidden');
      empty.classList.remove('hidden');
      empty.textContent = '这是拖动示教回放预览：详细参数见上方“拖动示教”卡片。';
      S.rendered.preview = preview.id + '|teaching';
      return;
    }
    const validity = preview.validity || { valid: false, reasons: ['预览不可用'], drift_deg: null };
    const signature = preview.id + '|' + preview.used + '|' + (validity.reasons || []).join(',');
    if (signature !== S.rendered.preview) {
      S.rendered.preview = signature;
      const body = $('preview-body');
      body.textContent = '';
      (payload.waypoints || []).forEach((row) => {
        const tr = el('tr');
        tr.appendChild(el('td', null, String(row.index)));
        tr.appendChild(el('td', null, view(row.move_s, 2)));
        tr.appendChild(el('td', null, view(row.arrival_s, 2)));
        tr.appendChild(el('td', null, view(row.end_s, 2)));
        (row.target_deg || []).forEach((value) => tr.appendChild(el('td', null, view(value, 3))));
        body.appendChild(tr);
      });
      $('preview-warnings').textContent = '';
      (payload.warnings || []).forEach((warning) => $('preview-warnings').appendChild(el('li', null, warning)));
      if (!validity.valid) {
        (validity.reasons || []).forEach((reason) => $('preview-warnings').appendChild(el('li', 'cell-bad', reason)));
      }
    }
    renderFlags($('preview-meta'), [
      ['票根', preview.id.slice(0, 12) + '…'],
      ['校验状态', validity.valid ? '有效' : '已失效', validity.valid ? 'cell-ok' : 'cell-bad'],
      ['剩余有效期', view(preview.remaining_s, 1) + ' s'],
      ['源坐标模式', payload.source_mode === 'absolute_deg' ? '软件绝对角度' : '相对本次起点'],
      ['执行坐标模式', '冻结为软件绝对角度'],
      ['预览起点', (payload.start_deg || []).map((value) => view(value, 2)).join(' / ') + ' °'],
      ['计划总时长', view(payload.total_s, 2) + ' s'],
      ['时间预算上限', view(payload.budget && payload.budget.plan_limit_seconds, 1) + ' s（含保持与 8 秒余量）'],
      ['终点回到起点', bool(payload.returns_to_start)],
      ['起点漂移', validity.drift_deg === null ? '—' : view(validity.drift_deg, 2) + ' °'],
    ]);
  }

  function renderOperation(data) {
    const operation = data.operation;
    const panel = $('operation-panel');
    const empty = $('operation-empty');
    if (!operation) {
      panel.classList.add('hidden');
      empty.classList.remove('hidden');
      empty.textContent = '尚未执行操作。';
      S.rendered.operation = null;
      return;
    }
    panel.classList.remove('hidden');
    empty.classList.add('hidden');
    const signature = operation.id + '|' + operation.phase + '|' + operation.exit_code + '|'
      + (operation.stdout_tail || []).length + '|' + (operation.result ? operation.result.accuracy_passed : 'x');
    if (signature !== S.rendered.operation) {
      S.rendered.operation = signature;
      renderOperationBody(operation);
    }
  }

  function renderOperationBody(operation) {
    const result = operation.result;
    const kind = operation.kind === 'gravity_float' ? '重力补偿自由拖动' :
      operation.kind === 'drag_teaching' ? '拖动示教记录（自由拖动）' :
        operation.kind === 'teaching_playback' ? '拖动示教回放（正常位置控制）' :
          (operation.kind === 'hold' ? '当前位置有限保持' : '路点计划执行');
    const phases = { starting: '启动中', running: '运行中', stopping: '停止中', stopped: '已停止',
      finished: '完成', failed: '失败' };
    renderFlags($('operation-summary'), [
      ['操作 ID', operation.id],
      ['类型', kind],
      ['阶段', phases[operation.phase] || operation.phase],
      ['退出码', operation.exit_code === null ? '—' : String(operation.exit_code)],
      ['开始', clockOf(operation.started_at)],
      ['结束', clockOf(operation.finished_at)],
      ['停止原因', operation.stop_reason || '—'],
      ['心跳租约过期', bool(operation.lease_expired)],
      ['源计划 SHA256', operation.source_sha256 ? operation.source_sha256.slice(0, 16) + '…' : '—'],
      ['冻结计划 SHA256', operation.plan_sha256 ? operation.plan_sha256.slice(0, 16) + '…' : '—'],
    ]);

    const action = result || {};
    const calibrationBox = $('calibration-results');
    calibrationBox.textContent = '';
    if (action.calibration) {
      const c = action.calibration;
      calibrationBox.appendChild(el('p', c.campaign_complete ? 'cell-ok' : 'cell-bad',
        '重力采样 ' + c.group + '：合格 ' + c.qualified_points + '/' + c.requested_points
        + '；' + (c.campaign_complete ? '本组采样完成' : '本组未完成')
        + '。这不是全姿态标定完成，α未改变。'));
      (c.points || []).forEach((point) => {
        const pose = (point.actual_deg || []).map((q) => view(q, 2)).join(', ');
        calibrationBox.appendChild(el('p', point.qualified ? 'cell-ok' : 'cell-bad',
          '点' + point.index + '：' + (point.qualified
            ? ('合格，静态窗口 ' + view(point.span_s, 2) + ' 秒；实测角度 [' + pose + ']°')
            : ('未采到合格数据：' + point.reason))));
      });
      (c.paired_targets || []).forEach((pair) => {
        calibrationBox.appendChild(el('p', 'note',
          '配对 ' + pair.pair_id + ' 双向均值 [' + pair.midpoint_nm.map((q) => view(q, 3)).join(', ')
          + '] N·m，来向半差 [' + pair.half_difference_nm.map((q) => view(q, 3)).join(', ')
          + '] N·m，实测姿态差 [' + pair.pose_difference_deg.map((q) => view(q, 3)).join(', ') + ']°'));
      });
      calibrationBox.appendChild(el('p', 'note', '原始记录：' + c.raw_file));
    }
    if (action.teaching_playback) {
      const playback = action.teaching_playback;
      const tracking = action.playback_tracking || {};
      calibrationBox.appendChild(el('p', 'cell-ok',
        '拖动示教回放：速度比例 ' + view(playback.speed_factor, 2)
        + '，接近 ' + view(playback.approach_s, 2) + ' s + 记录 '
        + view((playback.retimed_duration_s || 0) - (playback.approach_s || 0), 2)
        + ' s，峰值 ' + view(playback.peak_speed_rad_s, 3) + ' / '
        + view(playback.limit_rad_s, 2) + ' rad/s。'));
      if (Array.isArray(playback.ranges_deg)) {
        calibrationBox.appendChild(el('p', 'note', '回放关节范围 °：'
          + playback.ranges_deg.map((row) => '[' + view(row[0], 2) + ', ' + view(row[1], 2) + ']').join(' ')));
      }
      if (tracking && Array.isArray(tracking.max_tracking_error_deg)) {
        calibrationBox.appendChild(el('p', 'note', '最大跟踪误差 °：'
          + tracking.max_tracking_error_deg.map((value) => view(value, 3)).join(' / ')
          + '（不是 0.5° 精度验收）'));
      }
    }
    renderFlags($('operation-action'), [
      ['标准 JTC 动作', result === null ? '无结果' :
        (['current_pose_hold', 'gravity_float'].includes(action.mode) ? '未调用（独立有限模式）' :
          (action.action_passed ? '错误码 0（成功）' : '未通过')),
        result === null ? 'cell-bad' : (action.action_passed ? 'cell-ok' : '')],
      ['原始错误码', action.trajectory_error_code === undefined || action.trajectory_error_code === null
        ? '—' : String(action.trajectory_error_code)],
      ['原始错误消息', action.trajectory_error_string || '—'],
      ['结果原因', action.reason || operation.result_error || '—'],
      ['阶段判定', action.phase_result || '—'],
      ['使能请求', action.arm_requested ? '已请求' : '未请求'],
      ['轨迹请求', action.trajectory_requested ? '已请求' : '未请求'],
      ['轨迹已开始', action.trajectory_started ? '是' : '否'],
      ['停止请求', action.stop_requested ? '已请求' : '未请求'],
      ['停止判定', action.stop_result || '—'],
      ['预检查原因', (action.prearm_reasons || []).join('；') || '—'],
      ['UNKNOWN 观测', action.unknown_observed ? '是（保持支撑并现场切电）' : '否',
        action.unknown_observed ? 'cell-bad' : ''],
      ['并发控制错误', action.concurrent_control_error || '—'],
      ['停止错误', action.stop_error || '—'],
      ['后端接管回执', action.backend_arm_receipt || '—'],
      ['动作中止（停止后写入）', action.stop_observations_confirmed ? '已确认' : '未确认',
        action.stop_observations_confirmed ? 'cell-ok' : 'cell-bad'],
    ]);

    const body = $('accuracy-body');
    body.textContent = '';
    const rows = action.waypoint_results || [];
    if (!rows.length) {
      const tr = el('tr');
      const cell = el('td', 'cell-missing', result === null ? '无结果：客户端未执行或提前退出' : '该操作没有逐点精度数据');
      cell.colSpan = 11;
      tr.appendChild(cell);
      body.appendChild(tr);
    } else {
      rows.forEach((row, index) => {
        const tr = el('tr');
        tr.appendChild(el('td', null, String(index + 1)));
        tr.appendChild(el('td', null, view(row.arrival_s, 2)));
        tr.appendChild(el('td', null, view(row.end_s, 2)));
        tr.appendChild(el('td', null, row.samples === undefined || row.samples === null ? '—' : String(row.samples)));
        tr.appendChild(el('td', null, view(row.sampled_span_s, 3)));
        const errors = row.max_error_deg || [];
        for (let axis = 0; axis < AXIS_COUNT; axis += 1) {
          tr.appendChild(el('td', null, view(errors[axis], 3)));
        }
        tr.appendChild(el('td', row.passed ? 'cell-ok' : 'cell-bad', row.passed ? '通过' : '未通过'));
        body.appendChild(tr);
      });
    }

    const accuracy = action.accuracy_passed;
    renderFlags($('operation-stop'), [
      ['逐点精度判据', accuracy === null || accuracy === undefined ? '不适用（无路点）'
        : (accuracy ? '全部通过' : '未通过'), accuracy === true ? 'cell-ok' : accuracy === false ? 'cell-bad' : ''],
      ['动作成功 ≠ 精度通过', action.action_passed && accuracy === false ? '动作成功但精度未通过' : '—',
        action.action_passed && accuracy === false ? 'cell-bad' : ''],
      ['客户端新失能确认', action.stop_observations_confirmed ? '已确认' : '未确认',
        action.stop_observations_confirmed ? 'cell-ok' : 'cell-bad'],
      ['面板停止状态', dataStopLabel()],
      ['面板失能观察', bool(S.data && S.data.ros && S.data.ros.disabled_observed)],
    ]);

    const stdout = $('operation-stdout');
    stdout.textContent = (operation.stdout_tail || []).join('\n') || '（无输出）';
  }

  function dataStopLabel() {
    return S.data && S.data.stop ? S.data.stop.label : '—';
  }

  function renderCapabilities(data) {
    const caps = data.capabilities || {};
    const cap = (name) => caps[name] || { ok: false, message: '状态未知' };
    $('cap-preview').textContent = cap('preview').ok ? '' : ('不可预览：' + cap('preview').message);
    let executeMessage = cap('execute').message;
    if (S.dirty) executeMessage = '计划已修改：请重新预览';
    else if (!S.ticket && cap('execute').ok) executeMessage = '请先在服务器预览以取得一次性票根';
    else if (S.ticket && !S.dirty && !allConfirmed()) executeMessage = '勾选三项确认后可执行';
    $('cap-execute').textContent = '执行：' + executeMessage;
    $('cap-hold').textContent = '保持：' + cap('hold').message;
    $('cap-reset').textContent = '复位：' + cap('reset').message;
    updateButtons();
  }

  function updateButtons() {
    const data = S.data;
    const caps = (data && data.capabilities) || {};
    const connected = S.connected;
    const busy = !!(data && data.operation && data.operation.running);
    $('btn-preview').disabled = !connected || busy || !(caps.preview && caps.preview.ok);
    $('btn-execute').disabled = !connected || busy || S.dirty || !S.ticket || !allConfirmed()
      || !(caps.execute && caps.execute.ok);
    $('btn-hold').disabled = !connected || busy || !allConfirmed() || !(caps.hold && caps.hold.ok);
    const driver = ((data && data.gravity_shadow) || {}).driver || {};
    $('btn-gravity-float').disabled = $('btn-hold').disabled || !driver.connected ||
      !driver.float_available || !$('ack-gravity-float').checked;
    $('btn-reset').disabled = !connected || busy || !(caps.reset && caps.reset.ok);
    $('btn-read').disabled = !connected || !(data && data.axes);
    const teaching = (data && data.teaching) || {};
    const tCaps = teaching.capabilities || {};
    const tOk = (name) => !!(tCaps[name] && tCaps[name].ok);
    const recording = !!(teaching.recording && ['starting', 'ramp_in', 'recording'].includes(teaching.phase));
    $('float-kd').disabled = busy || recording || !(data && data.ros && data.ros.stage === 0);
    $('float-seconds').disabled = $('float-kd').disabled;
    const hasComplete = !!(teaching.last && teaching.last.complete === true && !teaching.last.saved);
    $('btn-teaching-start').disabled = !connected || busy || recording || !tOk('start')
      || !$('ack-drag-teaching').checked || !allConfirmed();
    $('btn-teaching-stop').disabled = !connected || !recording;
    $('btn-teaching-save').disabled = !connected || recording || !hasComplete;
    $('btn-teaching-load').disabled = !connected || !$('teaching-select').value;
    $('btn-teaching-preview').disabled = !connected || busy || recording || !$('teaching-select').value
      || !tOk('preview');
    const teachingTicketLive = !!(data && data.preview && data.preview.mode === 'teaching'
      && !data.preview.used);
    if (!teachingTicketLive) S.teaching.ticket = null;
    $('btn-teaching-execute').disabled = !connected || busy || !S.teaching.ticket
      || !tOk('execute') || !allConfirmed();
    const capture = (data && data.supported_capture) || {};
    const cCaps = capture.capabilities || {};
    const cOk = (name) => !!(cCaps[name] && cCaps[name].ok);
    const captureCurrent = capture.current || {};
    const captureTicketLive = !!(data && data.preview && data.preview.mode === 'supported_capture'
      && !data.preview.used);
    if (!captureTicketLive) S.capture.ticket = null;
    $('btn-capture-preview').disabled = !connected || busy || !cOk('preview');
    $('btn-capture-start').disabled = !connected || busy || !S.capture.ticket || !captureTicketLive
      || !cOk('start') || !$('ack-capture-start').checked || !allConfirmed();
    $('btn-capture-ack').disabled = !connected || !captureCurrent.id || captureCurrent.ack_sent
      || !cOk('ack') || !$('ack-operator-disengaged').checked;
    $('btn-stop').disabled = false;
  }

  function renderEvents(data) {
    const events = data.events || [];
    const signature = events.length + '|' + (events.length ? events[events.length - 1].message : '');
    if (signature === S.rendered.events) return;
    S.rendered.events = signature;
    const list = $('event-list');
    list.textContent = '';
    events.slice().reverse().forEach((event) => {
      const item = el('li');
      item.appendChild(el('span', 't', clockOf(event.at)));
      item.appendChild(el('span', event.level === 'warn' ? 'warn' : event.level === 'error' ? 'error' : null,
        event.message));
      list.appendChild(item);
    });
  }

  function renderRosDetail(data) {
    const ros = data.ros || {};
    const services = data.services || {};
    const speed = data.server && finite(data.server.reference_speed_rad_s)
      ? Number(data.server.reference_speed_rad_s) : null;
    $('speed-limit').textContent = speed === null ? '—' : String(speed);
    const serviceText = (name) => {
      const value = services[name];
      if (!value || !value.at) return '未调用';
      return (value.ok ? '接受' : '失败/拒绝') + ' @ ' + clockOf(value.at) + '（' + (value.message || '') + '）';
    };
    renderFlags($('ros-detail'), [
      ['ROS_DOMAIN_ID', (data.server && data.server.ros_domain_id) || '—'],
      ['存档目录', (data.server && data.server.archive_dir) || '—'],
      ['计划版本', (data.server && data.server.plan_schema) || '—'],
      ['受控轴', (data.server && data.server.joint_names || []).join(' / ') || '—'],
      ['受保护客户端', ros.client_script ? ros.client_script.split('/').pop() + ' · '
        + String(ros.client_sha256 || '').slice(0, 12) + '…' : (ros.client_error || '—')],
      ['客户端轴数', ros.client_axes === undefined ? '—' : String(ros.client_axes) + ' 轴'],
      ['限位文件', ros.limits_source ? '…/' + ros.limits_source.split('/').slice(-2).join('/') : '—'],
      ['stop_all', serviceText('stop_all')],
      ['reset_disabled', serviceText('reset_disabled')],
      ['switch_controller', serviceText('switch_controller')],
      ['会话预算', view(data.server && data.server.session_budget_seconds, 0) + ' s（驱动 180 s 上限，含停止余量）'],
      ['心跳超时', view(data.server && data.server.heartbeat_timeout_s, 1) + ' s'],
      ['预览票根有效期', view(data.server && data.server.preview_ttl_s, 0) + ' s'],
    ]);
  }

  function renderGravityMode(data) {
    const mode = data.gravity_mode || {};
    const request = mode.request || {};
    const body = $('gravity-mode-body');
    body.textContent = '';
    (mode.axes || []).forEach((row) => {
      const tr = el('tr');
      tr.appendChild(el('td', null, row.label || '—'));
      tr.appendChild(el('td', null, view(row.actual_deg, 3)));
      tr.appendChild(el('td', null, view(row.target_deg, 3)));
      const error = finite(row.error_deg) ? row.error_deg : null;
      tr.appendChild(el('td', error !== null && Math.abs(error) > 4 ? 'cell-bad' : null,
        error === null ? '—' : view(error, 3)));
      body.appendChild(tr);
    });
    $('gravity-mode-state').textContent = mode.label || '未知';
    $('gravity-mode-reason').textContent = (mode.reason || '') + ' '
      + (request.message ? '请求：' + request.message + ' ' : '')
      + (mode.session_end || '');
    renderFlags($('gravity-mode-flags'), [
      ['模式', mode.label || '未知'],
      ['已确认锁定', mode.locked_confirmed ? '是' : '否'],
      ['保持目标/实际误差', (mode.axes || []).map((row) => (row.label || '') + ' '
        + view(row.target_deg, 2) + '° / ' + view(row.error_deg, 2) + '°').join('；')],
      ['当前增益 Kp / Kd', view(mode.kp, 1) + ' / ' + view(mode.kd, 1)],
      ['增益渐入比例', view(mode.gain_scale, 2)],
      ['本次会话剩余', finite(mode.remaining_s) ? view(mode.remaining_s, 1) + ' s' : '—'],
      ['驱动锁定状态', viewInt(mode.lock_state)],
      ['请求状态', request.phase || '—'],
      ['结束动作', mode.session_end || '全轴停止/失能确认'],
    ]);
    const lease = data.lease || {};
    const operation = data.operation || {};
    const busy = mode.request && (mode.request.phase === 'queued' || mode.request.phase === 'sent');
    $('btn-gravity-lock').disabled = !(mode.phase === 'float') || !lease.fresh
      || !operation.running || !mode.hold_ready || !!busy;
    $('btn-gravity-resume').disabled = !(mode.phase === 'locked' || mode.phase === 'releasing')
      || !lease.fresh || !operation.running || !!busy;
  }

  function renderGravity(data) {
    const shadow = data.gravity_shadow || {};
    const body = $('gravity-body');
    body.textContent = '';
    const available = shadow.available === true;
    if (!available) {
      const tr = el('tr');
      const td = el('td', 'cell-missing', shadow.reason || '重力 shadow 不可用');
      td.colSpan = 11;
      tr.appendChild(td);
      body.appendChild(tr);
    } else {
      (shadow.rows || []).forEach((row) => {
        const tr = el('tr');
        tr.appendChild(el('td', null, (row.joint || '').replace('slaihand_joint', 'J')));
        tr.appendChild(el('td', null, view(row.q_deg, 3)));
        tr.appendChild(el('td', null, view(row.tau_model_nm, 4)));
        tr.appendChild(el('td', null, view(row.tau_protocol_nm, 4)));
        tr.appendChild(el('td', finite(row.p_term_nm) ? null : 'cell-missing', view(row.p_term_nm, 3)));
        tr.appendChild(el('td', null, view(row.d_term_nm, 3)));
        tr.appendChild(el('td', finite(row.applied_ff_nm) ? null : 'cell-missing', view(row.applied_ff_nm, 3)));
        const appliedHeadroom = row.applied_headroom_nm;
        tr.appendChild(el('td', finite(appliedHeadroom)
          ? (appliedHeadroom <= 0 ? 'cell-bad' : 'cell-ok') : 'cell-missing',
        row.guard_enabled === false ? '门槛已关闭' : finite(appliedHeadroom) ? view(appliedHeadroom, 3) : '未知'));
        tr.appendChild(el('td', finite(row.predicted_ff_nm) ? null : 'cell-missing', view(row.predicted_ff_nm, 3)));
        const predictedHeadroom = row.predicted_headroom_nm;
        tr.appendChild(el('td', finite(predictedHeadroom)
          ? (predictedHeadroom <= 0 ? 'cell-bad' : 'cell-ok') : 'cell-missing',
        row.guard_enabled === false ? '门槛已关闭' : finite(predictedHeadroom) ? view(predictedHeadroom, 3) : '未知'));
        tr.appendChild(el('td', null, row.guard_enabled === false ? '已关闭' : view(row.guard_nm, 0)));
        body.appendChild(tr);
      });
    }
    const referenceNote = shadow.reference_known
      ? '控制器参考新鲜：P 项与“已应用余量”为实际计算值。'
      : '控制器参考未知：P 项与总余量显示为“未知”，不以零误差假设填充。';
    $('gravity-reason').textContent = available
      ? '模式 ' + (shadow.mode || 'shadow')
        + '；假设补偿预测 α_pred=' + view(shadow.shadow_alpha, 2) + '（仅模型预测，未写入命令）。'
        + referenceNote + ' ' + ((shadow.warnings || []).join(' '))
      : '估算不可用：' + (shadow.reason || '未知原因');
    const metadata = shadow.metadata || {};
    const config = shadow.config || {};
    const driver = shadow.driver || {};
    const gravity = config.gravity_base_m_s2 || metadata.gravity_base_m_s2 || [];
    const j3 = config.j3_offset_rad !== undefined ? config.j3_offset_rad : 0;
    const driverNote = driver.connected
      ? (driver.active
        ? '驱动前馈活动：α=' + view(driver.alpha, 2) + '，已写入 '
          + (driver.applied || []).map((v) => view(v, 3)).join(' / ') + ' N·m。'
        : '驱动前馈未活动（阶段 ' + viewInt(driver.stage) + '）；历史最后写入 '
          + (driver.last_written || []).map((v) => view(v, 3)).join(' / ') + ' N·m。')
      : '驱动未报告前馈状态（' + (driver.reason || '未连接') + '）。';
    $('gravity-reason').textContent += ' ' + driverNote;
    renderFlags($('gravity-flags'), [
      ['模型资格', shadow.provisional ? 'PROVISIONAL（仅诊断，不能拟合/正式补偿）' : 'QUALIFIED',
        shadow.provisional ? 'cell-bad' : 'cell-ok'],
      ['正式重力', shadow.formal ? '允许' : '拒绝', shadow.formal ? 'cell-ok' : 'cell-bad'],
      ['模型轴数', viewInt(shadow.model_axes) + ' / 计划 ' + viewInt(shadow.plan_axes)],
      ['模式', shadow.mode === 'shadow' ? 'shadow（只显示）' : (shadow.mode || '—')],
      ['前馈已应用', driver.connected ? bool(driver.active) : '未知'],
      ['后端', metadata.backend || '—'],
      ['模型 URDF', (metadata.urdf_sha256 || '—').slice(0, 12)],
      ['模型参数档案', (metadata.mass_profile_status || '—') + '（' + (metadata.mass_profile_sha256 || '').slice(0, 12) + '）'],
      ['参数形式', metadata.parameterization === 'identified_base' ? '实测重力基参数' : '质量和重心'],
      ['总质量', metadata.parameterization === 'identified_base' ? '未辨识' : view(metadata.total_mass_kg, 3) + ' kg'],
      ['重力方向（基础系）', gravity.join(', ') + ' m/s²'],
      ['J3 固定偏置', view(j3, 4) + ' rad'],
      ['主机力矩门槛', (metadata.guards_nm || []).map(v => v > 0 ? v + ' N·m' : '已关闭').join(' / ')],
      ['kp / kd', view(metadata.kp, 0) + ' / ' + view(metadata.kd, 0)],
      ['驱动前馈活动', driver.connected ? (driver.active ? '是（阶段 ' + viewInt(driver.stage) + '）' : '否') : '未接入'],
      ['驱动控制模式', driver.gravity_hold ? '重力位置保持（已锁定）' : driver.gravity_float
        ? '自由拖动，Kp=0 / Kd=' + view(driver.float_kd, 1) : '位置保持 / 轨迹'],
      ['驱动前馈 α / 斜坡', driver.connected ? view(driver.alpha, 2) + ' / ' + view(driver.ramp, 2) + ' s' : '—'],
      ['驱动已写入值 N·m', driver.connected ? (driver.applied || []).map((v) => view(v, 3)).join(' / ') : '—'],
      ['历史最后写入 N·m', driver.connected ? (driver.last_written || []).map((v) => view(v, 3)).join(' / ') : '—'],
      ['驱动状态时效', driver.connected ? view(driver.age_s, 2) + ' s' : '—'],
      ['前馈协议', driver.protocol ? driver.protocol + ' · ' + viewInt(driver.axes) + ' 轴/持锁数组 '
        + ((driver.hold_target || []).length || 0) + ' 维'
        + (driver.protocol_source === 'layout' ? '（显式元数据）' : '（按长度推断）') : '—',
        driver.error ? 'cell-bad' : ''],
      ['前馈协议错误', driver.error ? driver.error : '无', driver.error ? 'cell-bad' : ''],
    ]);
  }

  function render(data) {
    const connection = data.connection || {};
    $('connection-status').textContent = connection.message || '可检查连接';
    $('btn-reconnect').disabled = connection.phase === 'connecting' || !!(data.operation && data.operation.running);
    renderChips(data);
    renderBanner(data);
    renderAxisSummary(data);
    renderAxes(data);
    renderMotionFlags(data);
    renderDiagnostics(data);
    renderMargin(data);
    renderGravity(data);
    renderGravityMode(data);
    renderTeaching(data);
    renderCapture(data);
    renderAxisCalibration(data);
    renderPreview(data);
    renderOperation(data);
    renderCapabilities(data);
    renderEvents(data);
    renderRosDetail(data);
  }

  function renderDisconnected() {
    const summary = $('axis-summary');
    if (summary) summary.textContent = '面板通信断开；不显示缓存角度。';
    $('chip-ros').textContent = '反馈：面板断开';
    $('chip-ros').className = 'chip chip-bad';
    $('btn-preview').disabled = true;
    $('btn-execute').disabled = true;
    $('btn-hold').disabled = true;
    $('btn-gravity-float').disabled = true;
    if ($('btn-gravity-lock')) $('btn-gravity-lock').disabled = true;
    if ($('btn-gravity-resume')) $('btn-gravity-resume').disabled = true;
    $('btn-teaching-start').disabled = true;
    $('btn-teaching-stop').disabled = true;
    $('btn-teaching-save').disabled = true;
    $('btn-teaching-load').disabled = true;
    $('btn-teaching-preview').disabled = true;
    $('btn-teaching-execute').disabled = true;
    $('btn-capture-preview').disabled = true;
    $('btn-capture-start').disabled = true;
    $('btn-capture-ack').disabled = true;
    if ($('btn-axis-cal-start')) $('btn-axis-cal-start').disabled = true;
    if ($('btn-axis-cal-abort')) $('btn-axis-cal-abort').disabled = true;
    ['zero', 'positive', 'lower', 'upper'].forEach((phase) => {
      if ($('btn-axis-cal-' + phase)) $('btn-axis-cal-' + phase).disabled = true;
    });
    $('btn-reset').disabled = true;
    $('btn-stop').disabled = false;
  }

  async function poll() {
    if (S.polling) return;
    S.polling = true;
    const started = performance.now();
    try {
      const data = await api('GET', '/api/status');
      S.pollMs = performance.now() - started;
      S.connected = true;
      S.data = data;
      if (S.lastPollError) S.lastPollError = null;
      render(data);
    } catch (error) {
      S.connected = false;
      renderDisconnected();
      const text = '面板服务断开或会话失效：' + describeError(error);
      if (S.lastPollError !== text) {
        S.lastPollError = text;
        setMessage(text + '（已自动重试；若已停止操作，请到终端确认）', 'error');
      }
    } finally {
      S.polling = false;
    }
  }

  async function heartbeat() {
    try {
      await api('POST', '/api/heartbeat', {});
    } catch (error) {
      /* 轮询会报告连接问题；心跳失败本身不重复提示 */
    }
  }

  function bind() {
    document.querySelectorAll('.tab').forEach((button) => {
      button.addEventListener('click', () => setTab(button.dataset.tab));
    });
    $('btn-calibration-group').addEventListener('click', async () => {
      const name = $('calibration-group').value;
      const fiveAxisPlan = name.startsWith('WIDE_');
      if (S.data && S.data.unified && S.data.unified.axes === 5 && !fiveAxisPlan) {
        setMessage('五轴模式下旧四轴标定组（schema=slaihand_joint_plan/1）不可载入，'
          + '也不会被补成 J5；请等待正式五轴标定组与 J5 资格。', 'warn');
        return;
      }
      try {
        const response = await api('GET', '/api/calibration/groups');
        if (fiveAxisPlan) {
          // Position-only candidate, not a published formal gravity group.
          applyPlan(response.groups[name], name, null);
          $('execution-mode').value = 'trajectory';
          $('ack-free-static').checked = false;
          markDirty();
          setMessage(name + ' 已载入为普通五轴位置轨迹（相对起点，非正式重力采样）。'
            + '请先在“轨迹跟随”模式预览；不会自动拟合或开启重力补偿。', 'warn');
          return;
        }
        applyPlan(response.groups[name], name, name);
        $('execution-mode').value = 'calibration';
        $('ack-free-static').checked = false;
        markDirty();
        setMessage(name + ' 已载入：程序自动到位、等待稳定并逐点采样；任一点失败停止。请先预览完整路径，再确认执行。尚未使能。', 'warn');
      } catch (error) { setMessage(describeError(error), 'error'); }
    });
    $('btn-connection-check').addEventListener('click', doConnectionCheck);
    $('btn-reconnect').addEventListener('click', doReconnect);
    $('execution-mode').addEventListener('change', markDirty);
    $('btn-stop').addEventListener('click', doStop);
    $('btn-preview').addEventListener('click', doPreview);
    $('btn-execute').addEventListener('click', doExecute);
    $('btn-hold').addEventListener('click', doHold);
    $('btn-gravity-float').addEventListener('click', doGravityFloat);
    $('btn-gravity-lock').addEventListener('click', doGravityLock);
    $('btn-gravity-resume').addEventListener('click', doGravityResume);
    $('btn-teaching-start').addEventListener('click', doTeachingStart);
    $('btn-teaching-stop').addEventListener('click', doTeachingStop);
    $('btn-teaching-save').addEventListener('click', doTeachingSave);
    $('float-kd').addEventListener('input', () => {
      $('float-kd-value').textContent = Number($('float-kd').value).toFixed(1);
    });
    $('btn-teaching-load').addEventListener('click', doTeachingLoad);
    $('btn-teaching-preview').addEventListener('click', doTeachingPreview);
    $('btn-teaching-execute').addEventListener('click', doTeachingExecute);
    $('btn-capture-preview').addEventListener('click', doCapturePreview);
    $('btn-capture-start').addEventListener('click', doCaptureStart);
    $('btn-capture-ack').addEventListener('click', doCaptureAck);
    $('btn-axis-cal-start').addEventListener('click', doAxisCalStart);
    $('btn-axis-cal-abort').addEventListener('click', doAxisCalAbort);
    AXIS_CAL_PHASES.forEach((phase) => {
      $('btn-axis-cal-' + phase).addEventListener('click', () => doAxisCalCapture(phase));
    });
    ['ack-axis-cal-supported', 'ack-axis-cal-clearance'].forEach((id) => {
      $(id).addEventListener('change', () => updateAxisCalButtons(S.data));
    });
    $('teaching-select').addEventListener('change', () => {
      if ($('teaching-select').value) loadTeaching($('teaching-select').value);
    });
    $('btn-reset').addEventListener('click', doReset);
    $('btn-read').addEventListener('click', readCurrent);
    $('btn-add').addEventListener('click', () => {
      if (S.rows.length >= MAX_WAYPOINTS) {
        setMessage('最多 ' + MAX_WAYPOINTS + ' 个路点。', 'warn');
        return;
      }
      addRow([0, 0, 0, 0, 0], 4, 1);
      markDirty();
    });
    $('btn-remove-last').addEventListener('click', () => {
      if (S.rows.length <= 1) {
        setMessage('至少保留一个路点。', 'warn');
        return;
      }
      const row = S.rows[S.rows.length - 1];
      row.tr.remove();
      S.rows.pop();
      renumber();
      markDirty();
    });
    $('btn-export').addEventListener('click', exportJson);
    $('btn-import').addEventListener('click', () => $('file-import').click());
    $('file-import').addEventListener('change', (event) => {
      importJson(event.target.files && event.target.files[0]);
      event.target.value = '';
    });
    $('btn-save').addEventListener('click', savePlan);
    $('btn-load').addEventListener('click', loadSaved);
    document.querySelectorAll('input[name="coords"]').forEach((input) => {
      input.addEventListener('change', markDirty);
    });
    ['ack-supported', 'ack-clearance', 'ack-native', 'ack-free-static', 'ack-gravity-float',
      'ack-drag-teaching', 'ack-capture-start', 'ack-operator-disengaged'].forEach((id) => {
      $(id).addEventListener('change', updateButtons);
    });
    window.addEventListener('pagehide', () => {
      try {
        fetch('/api/lease/abandon', {
          method: 'POST',
          keepalive: true,
          credentials: 'same-origin',
          headers: { 'X-Panel-Token': TOKEN, 'Content-Type': 'application/json' },
          body: '{}',
        });
      } catch (error) {
        /* 心跳租约仍会兜底停止 */
      }
    });
  }

  function init() {
    buildAxes();
    bind();
    setTab('overview');
    resetEditor();
    addRow([0, 0, 0, 0, 0], 2, 1);
    $('mode-relative').checked = true;
    markDirty();
    $('editor-hint').textContent = '当前没有预览；编辑或读取当前位置后，请先在服务器预览。';
    setMessage('已连接页面；只用真实反馈显示状态，先“读取当前位置”或编辑路点，再做服务器预览。', 'ok');
    poll();
    heartbeat();
    setInterval(poll, 500);
    setInterval(heartbeat, 1000);
    loadSavedList();
    loadTeachingList();
  }

  init();
})();
