// Execute the real editor/result functions without ROS, a browser, or network.
// Rendering primitives are an in-memory DOM; no motion action is exposed.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const root = path.resolve(__dirname, '../../..');
const nodes = new Map();
class Node {
  constructor(id = '') { this.id = id; this.children = []; this._text = ''; this.value = ''; this._checked = false; }
  set textContent(value) { this._text = String(value); this.children = []; }
  get textContent() { return this._text + this.children.map(n => n.textContent).join(''); }
  appendChild(node) { this.children.push(node); return node; }
  addEventListener() {}
  set checked(value) {
    if (value && ['mode-absolute', 'mode-relative'].includes(this.id)) {
      for (const id of ['mode-absolute', 'mode-relative']) get(id)._checked = false;
    }
    this._checked = value;
  }
  get checked() { return this._checked; }
  getAttribute() { return 'offline-test-token'; }
}
function get(id) { if (!nodes.has(id)) nodes.set(id, new Node(id)); return nodes.get(id); }
get('mode-absolute').value = 'absolute_deg'; get('mode-relative').value = 'relative_start_deg';
const document = {
  getElementById: get, createElement: () => new Node(),
  querySelector(selector) {
    if (selector.startsWith('meta')) return new Node();
    if (selector.includes('coords')) return ['mode-absolute','mode-relative'].map(get).find(n => n.checked);
    throw Error('unexpected query: ' + selector);
  },
};
const file = path.join(root, 'src/slaihand_bringup/web/control_panel/panel.js');
let source = fs.readFileSync(file, 'utf8');
assert(source.includes('  init();'));
source = source.replace('  init();', `
  updateButtons = () => {}; // isolate editor/rendering from unrelated capability widgets
  globalThis.panelTest = {applyPlan, planFromEditor, renderOperationBody};
`);
const context = {document, console, window: {}, globalThis: {}};
vm.runInNewContext(source, context, {filename: file});
const api = context.globalThis.panelTest;
const plan = JSON.parse(fs.readFileSync(path.join(__dirname,
  'gravity_compensation_trajectories_20261109/GC01_J1_J2_PAIRED_ZERO.json'), 'utf8'));
api.applyPlan(plan, 'GC01 重力验证 J1/J2配对');
assert.equal(get('mode-absolute').checked, true, 'loading absolute GC plan must select absolute radio');
assert.equal(api.planFromEditor().ok, true);
get('mode-relative').checked = true;
assert.equal(api.planFromEditor().ok, false, 'GC plan must reject accidental relative mode');
get('mode-absolute').checked = true;
assert.equal(api.planFromEditor().ok, true);
for (const name of ['GC05 new sampling', 'GC06 new sampling', 'GC07 new sampling', 'GC08 new sampling']) {
  api.applyPlan(plan, name);
  assert.equal(get('execution-mode').value, 'trajectory');
  get('mode-relative').checked = true;
  assert.equal(api.planFromEditor().ok, false, name + ' must retain absolute-zero contract');
}
get('mode-absolute').checked = true;
get('waypoints-body').children.at(-1).children[1].children[0].value = '1';
assert.equal(api.planFromEditor().ok, false, 'GC final target must stay absolute zero');
api.applyPlan(plan, 'generic custom trajectory'); get('mode-relative').checked = true;
assert.equal(api.planFromEditor().ok, true, 'ordinary plans retain relative mode support');
const rows = [{arrival_s:8,end_s:14,samples:0,sampled_span_s:0,max_error_deg:[],passed:false},
              {arrival_s:22,end_s:28,samples:30,sampled_span_s:.29,max_error_deg:[1,0,0,0,0],passed:false},
              {arrival_s:36,end_s:42,samples:30,sampled_span_s:.29,max_error_deg:[0,0,0,0,0],passed:true}];
api.renderOperationBody({kind:'execute',id:'offline',phase:'failed',exit_code:1,stdout_tail:[],
  result:{waypoint_results:rows,accuracy_passed:false}});
const rendered = get('accuracy-body').children;
assert.equal(rendered[0].children.at(-1).textContent, '未采样');
assert.equal(rendered[1].children.at(-1).textContent, '未通过');
assert.equal(rendered[2].children.at(-1).textContent, '通过');
api.renderOperationBody({kind:'execute',id:'offline',phase:'failed',exit_code:1,stdout_tail:[],
  result:{waypoint_results:[rows[0]],accuracy_passed:false}});
assert(get('operation-stop').textContent.includes('未采到验收窗口'));
console.log('PASS: actual editor GC absolute-zero contract; generic relative plans; missing vs failing vs passing samples');
