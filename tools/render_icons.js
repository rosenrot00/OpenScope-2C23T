// Rasterise Lucide icons (ISC licence) at their exact on-screen pixel sizes.
// Output: one PNG per icon (white on transparent) for tools/gen_fonts.py.
//
//   node tools/render_icons.js <lucide-static/icons dir> <out dir>
const fs = require('fs');
const path = require('path');
const { chromium } = require('playwright');

// Our own shapes, drawn in Lucide's 24-unit grid and stroke style.
const CUSTOM = {
  // A straight-edged lightning bolt (Lucide's 'zap' is more slanted).
  bolt: '<svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" ' +
        'stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">' +
        '<path d="M13.5 2 4.5 13.5h6.5L9.5 22l10-12.5h-6.5L13.5 2z"/></svg>',
  // Menu tools.
  multimeter: svg24('<rect x="5" y="2" width="14" height="20" rx="2.5"/><rect x="8" y="5" width="8" height="5" rx="1"/>' +
                    '<circle cx="12" cy="15.5" r="2.5"/>'),
  scope: svg24('<rect x="2" y="4" width="20" height="16" rx="2.5"/>' +
               '<path d="M5 12c1.5-4.5 3-4.5 4.5 0s3 4.5 4.5 0 3-4.5 4.5 0"/>'),
  // The device's combined pause/play (OK) key, printed as ‖▶.
  playpause: '<svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 12 12">' +
             '<rect x="0.5" y="2" width="2" height="8" rx="0.4" fill="#ffffff"/>' +
             '<rect x="3.5" y="2" width="2" height="8" rx="0.4" fill="#ffffff"/>' +
             '<path d="M7 2v8l4.8-4z" fill="#ffffff"/></svg>',
  // Trigger edges: a step with an arrow on the rising / falling side.
  edge_rise: svg24('<path d="M3 19h7V5h11"/><path d="M7 11l3-3 3 3"/>'),
  edge_fall: svg24('<path d="M3 5h7v14h11"/><path d="M7 13l3 3 3-3"/>'),
  generator: svg24('<path d="M2 16h3.5V8h6v8h6V8H21"/><path d="M2 20h20" opacity="0.45"/>'),
};

// Multimeter key marks, matching the symbols printed on the device's keys.
// Drawn in their own grids; the ICONS entry gives the on-screen width and
// height, which keep each grid's aspect ratio.
function svgGrid(w, h, body) {
  return `<svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 ${w} ${h}" fill="none" ` +
         'stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">' + body + '</svg>';
}
CUSTOM.key_dc = svgGrid(24, 16, '<path d="M3 4h18"/><path d="M3 12h3.5"/><path d="M10.25 12h3.5"/><path d="M17.5 12h3.5"/>');
CUSTOM.key_ac = svgGrid(24, 16, '<path d="M2 8c2.5-6.5 7.5-6.5 10 0s7.5 6.5 10 0"/>');
CUSTOM.key_diode_cont = svgGrid(38, 24,
  '<path d="M1 12h6"/><path d="M7 6.5v11l9-5.5z" fill="#ffffff"/><path d="M16 6v12"/><path d="M16 12h4"/>' +
  '<circle cx="25" cy="12" r="1.2" fill="#ffffff" stroke="none"/>' +
  '<path d="M28 9a4 4 0 0 1 0 6"/><path d="M31 6.5a7.5 7.5 0 0 1 0 11"/><path d="M34 4a11 11 0 0 1 0 16"/>');
CUSTOM.key_cap = svgGrid(24, 24, '<path d="M2 12h7"/><path d="M15 12h7"/><path d="M9 5v14"/><path d="M15 5v14"/>');

function svg24(body) {
  return '<svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" ' +
         'stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">' + body + '</svg>';
}

// name in the firmware, icon (Lucide name or CUSTOM key), pixel size, stroke width (24-unit space)
const ICONS = [
  ['check_xl', 'check', 34, 3.0],
  ['zap_xl', 'bolt', 32, 2.0],
  ['alert_sm', 'triangle-alert', 12, 2.4],
  ['beep_sm', 'volume-2', 11, 2.4],
  ['pause_sm', 'pause', 9, 3.0],
  ['plug_sm', 'plug', 12, 2.4],
  ['charge_sm', 'bolt', 11, 2.4],
  ['usb_sm', 'usb', 12, 2.2],
  ['multimeter_md', 'multimeter', 26, 2.0],
  ['scope_md', 'scope', 26, 2.0],
  ['generator_md', 'generator', 26, 2.0],
  ['settings_md', 'settings', 26, 2.0],
  ['chevron_l_sm', 'chevron-left', 11, 2.6],
  ['chevron_r_sm', 'chevron-right', 11, 2.6],
  ['playpause_sm', 'playpause', 12, 0],
  ['move_sm', 'move', 11, 2.4],
  ['beep_md', 'volume-2', 16, 2.2],
  ['display_md', 'sun', 16, 2.2],
  ['start_md', 'power', 16, 2.2],
  ['sleep_md', 'moon', 16, 2.2],
  ['export_md', 'hard-drive-download', 16, 2.2],
  ['reset_md', 'rotate-ccw', 16, 2.2],
  ['info_md', 'info', 16, 2.2],
  ['storage_sm', 'hard-drive', 12, 2.2],
  ['ok_md', 'circle-check', 20, 2.2],
  ['error_md', 'circle-x', 20, 2.2],
  ['edge_rise_sm', 'edge_rise', 11, 2.6],
  ['edge_fall_sm', 'edge_fall', 11, 2.6],
  ['scope_sm', 'scope', 12, 2.2],
  ['generator_sm', 'generator', 12, 2.2],
  // name, icon, width, stroke, height (when not square)
  ['key_dc', 'key_dc', 16, 3.2, 11],
  ['key_ac', 'key_ac', 16, 3.2, 11],
  ['key_diode_cont', 'key_diode_cont', 32, 2.6, 20],
  ['key_cap', 'key_cap', 19, 2.8],
];

(async () => {
  const [src, out] = process.argv.slice(2);
  fs.mkdirSync(out, { recursive: true });
  const browser = await chromium.launch();
  const page = await browser.newPage({ deviceScaleFactor: 1 });
  for (const [name, icon, size, stroke, height] of ICONS) {
    let svg = CUSTOM[icon] || fs.readFileSync(path.join(src, icon + '.svg'), 'utf8');
    svg = svg.replace(/<!--[\s\S]*?-->/g, '')
      .replace(/width="24"/, `width="${size}"`).replace(/height="24"/, `height="${height || size}"`)
      .replace(/stroke="currentColor"/, 'stroke="#ffffff"')
      .replace(/stroke-width="[0-9.]+"/, `stroke-width="${stroke}"`);
    await page.setContent(`<html><body style="margin:0;background:transparent">${svg}</body></html>`);
    const el = await page.$('svg');
    await el.screenshot({ path: path.join(out, name + '.png'), omitBackground: true });
  }
  await browser.close();
})();
