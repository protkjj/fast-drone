// run.cjs가 저장한 시뮬 결과 JSON(.trace 있는 단일 실행)을 엑셀에서 바로
// 열리는 CSV로 바꾼다. sweep.cjs 요약(.trace 없음)은 대상이 아니다 --
// sweep은 조건당 다회 실행이라 "한 줄 = 한 타임스텝" CSV가 안 맞는다.
//
// 사용법: node export_csv.cjs <result.json> [output.csv]
//   출력 생략 시 입력과 같은 경로에 확장자만 .csv로 바꿔 쓴다.
//
// 예: node run.cjs selected hybrid truth 8 ramp /tmp/run.json
//     node export_csv.cjs /tmp/run.json /tmp/run.csv
const fs = require('node:fs');
const path = require('node:path');
const runtime = require('./runtime.js');

function main() {
  const [inputPath, outputArg] = process.argv.slice(2);
  if (!inputPath) {
    console.error('사용법: node export_csv.cjs <result.json> [output.csv]');
    process.exitCode = 1;
    return;
  }
  const result = JSON.parse(fs.readFileSync(inputPath, 'utf8'));
  const csv = runtime.traceToCSV(result);
  const outputPath = outputArg || inputPath.replace(/\.json$/i, '')+'.csv';
  fs.writeFileSync(outputPath, csv);
  const rows = result.trace.length;
  console.error(`${outputPath}: ${rows}행, 컬럼 ${runtime.CSV_COLUMNS.length}개 (${path.basename(inputPath)}에서 변환)`);
}

main();
