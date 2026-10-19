const assert = require('node:assert/strict');
const { test } = require('node:test');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

function batteryUi() {
  const elements = new Map();
  const element = id => {
    if (!elements.has(id)) elements.set(id, {
      value: '', textContent: '', className: '', hidden: false, disabled: false,
      style: {}
    });
    return elements.get(id);
  };
  const context = vm.createContext({ document: { getElementById: element } });
  const source = fs.readFileSync(path.join(__dirname, '../js/ui-controller.js'), 'utf8')
    .replace('export const UIController =', 'globalThis.UIController =');
  vm.runInContext(source, context);
  return { ui: context.UIController, element };
}

test('idle battery forecast uses the selected charge as its starting point', () => {
  const { ui, element } = batteryUi();
  ui.updateBattery(
    { soc_pct: 16, simulated: true, config: { reserve_pct: 15, warning_pct: 20 } },
    { accepted: true, distance_m: 16.4, soc_pct: 100, required_pct: 0.6,
      soc_after_pct: 99.4, warning: false },
    { state: 'stopped' }
  );

  assert.equal(element('batteryPct').textContent, '16.0 %');
  assert.match(element('batteryEstimate').textContent, /consommation 0\.6 %/);
  assert.match(element('batteryEstimate').textContent, /arrivée 15\.4 %/);
  assert.equal(element('batteryEstimate').className, 'warn');
});

test('idle battery forecast marks a route refused when its rebased arrival is below reserve', () => {
  const { ui, element } = batteryUi();
  ui.updateBattery(
    { soc_pct: 15, simulated: true, config: { reserve_pct: 15, warning_pct: 20 } },
    { accepted: true, distance_m: 16.4, soc_pct: 100, required_pct: 0.6,
      soc_after_pct: 99.4, warning: false },
    { state: 'stopped' }
  );

  assert.match(element('batteryEstimate').textContent, /arrivée 14\.4 %/);
  assert.match(element('batteryEstimate').textContent, /Mission refusée/);
  assert.equal(element('batteryEstimate').className, 'error');
});
