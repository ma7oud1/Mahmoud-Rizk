const CACHE='smart-scan-pro-v2';
const CORE=['./','./index.html','./crop-engine.js'];
self.addEventListener('install',e=>e.waitUntil(caches.open(CACHE).then(c=>c.addAll(CORE)).then(()=>self.skipWaiting())));
self.addEventListener('activate',e=>e.waitUntil(caches.keys().then(keys=>Promise.all(keys.filter(k=>k.startsWith('smart-scan-pro-')&&k!==CACHE).map(k=>caches.delete(k)))).then(()=>self.clients.claim())));
self.addEventListener('fetch',e=>{
  const req=e.request;if(req.method!=='GET')return;
  const url=new URL(req.url),same=url.origin===self.location.origin;
  const library=['cdn.jsdelivr.net','cdnjs.cloudflare.com'].includes(url.hostname);
  if(!same&&!library)return;
  e.respondWith(caches.match(req).then(hit=>hit||fetch(req).then(res=>{
    if(res&&res.ok){const copy=res.clone();caches.open(CACHE).then(c=>c.put(req,copy));}
    return res;
  }).catch(()=>{if(req.mode==='navigate')return caches.match('./index.html');throw new Error('Offline resource unavailable') })));
});