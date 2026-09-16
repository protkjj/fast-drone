// 실제 배포 HTML의 스크립트를 실행한다. DOM과 3D만 대체하며 물리/제어식은 그대로 쓴다.
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

function simulator() {
  const html = fs.readFileSync(path.join(__dirname, '../../results/flight_sim.html'), 'utf8');
  const script = [...html.matchAll(/<script>([\s\S]*?)<\/script>/g)]
    .map(match => match[1]).find(text => text.includes('function xdot('));
  const elements = new Map();
  const defaults = {spd: '83', alt: '200', wsp: '0', wdir: '90', rtf: '1', ctrl: 'lqr'};
  const document = {
    hidden: false,
    querySelector(selector) {
      if (!elements.has(selector)) elements.set(selector, {
        value: defaults[selector.slice(1)] ?? '', textContent: '', hidden: true,
        setAttribute() {}, addEventListener() {},
      });
      return elements.get(selector);
    },
  };
  // CI의 CPU 부하가 시간 진행 테스트를 흔들지 않도록 계산 예산 시계를 고정한다.
  const context = vm.createContext({document, console, performance:{now:()=>0},
    requestAnimationFrame() {}, setTimeout() {},
    THREE: {Vector3: class {}},
  });
  const source = script.slice(0, script.lastIndexOf('\n// 브라우저 초기화'));
  vm.runInContext(source, context);
  vm.runInContext('paint = () => {}; render3D = () => {}; reset();', context);
  return {run: expression => vm.runInContext(expression, context), context};
}
module.exports = {simulator};
