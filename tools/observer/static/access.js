// A file:// preview has no helper, UART or HTTP event stream. State this before
// module loading (which browsers may reject for local files), without starting
// any board service or making the preview masquerade as a live portal.
if (location.protocol === 'file:') {
  const notice=document.getElementById('accessNotice');
  notice.hidden=false;
  notice.textContent='当前是本地文件预览，不能连接硬件。请明确启动设备上的检测站，再用 http://172.19.8.218:8766/ 打开门户。';
  document.getElementById('connection').textContent='本地文件 · 无硬件连接';
  document.querySelectorAll('.hardware, #connectVideo').forEach(el=>el.disabled=true);
}
