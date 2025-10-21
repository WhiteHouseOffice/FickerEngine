
const fs = require('fs');
const path = require('path');
const puppeteer = require('puppeteer');

(async () => {
  const outDir = path.join('out','web_frames');
  fs.mkdirSync(outDir, {recursive:true});
  const browser = await puppeteer.launch({
    headless: true,
    args: ['--no-sandbox','--enable-unsafe-webgpu']
  });
  const page = await browser.newPage();
  const url = 'file://' + path.resolve('webbuild/site/index.html');
  await page.goto(url);
  await page.waitForFunction(
  () => (window.engineReady === true) || document.readyState === 'complete',
  { timeout: 60000 }
  );
  for (let i=0;i<60;i++) {
    await page.evaluate(() => window.stepOnce && window.stepOnce());
    const buf = await page.screenshot({type:'png'});
    fs.writeFileSync(path.join(outDir, `frame_${String(i).padStart(3,'0')}.png`), buf);
  }
  await browser.close();
})().catch(e => { console.error(e); process.exit(1); });
