/* SMART SCAN PRO - Offline document edge detector
   Pure JavaScript Sobel + Hough transform. No CDN, WASM or network required. */
(function(){
  'use strict';
  function detect(img){
    const maxSide=640, scale=Math.min(1,maxSide/Math.max(img.naturalWidth||img.width,img.naturalHeight||img.height));
    const w=Math.max(80,Math.round((img.naturalWidth||img.width)*scale)),h=Math.max(80,Math.round((img.naturalHeight||img.height)*scale));
    const canvas=document.createElement('canvas');canvas.width=w;canvas.height=h;
    const ctx=canvas.getContext('2d',{willReadFrequently:true});ctx.drawImage(img,0,0,w,h);
    const rgba=ctx.getImageData(0,0,w,h).data,gray=new Float32Array(w*h),mag=new Float32Array(w*h);
    for(let i=0,j=0;i<gray.length;i++,j+=4)gray[i]=.299*rgba[j]+.587*rgba[j+1]+.114*rgba[j+2];
    const values=[];
    for(let y=1;y<h-1;y++)for(let x=1;x<w-1;x++){
      const i=y*w+x;
      const gx=-gray[i-w-1]+gray[i-w+1]-2*gray[i-1]+2*gray[i+1]-gray[i+w-1]+gray[i+w+1];
      const gy=-gray[i-w-1]-2*gray[i-w]-gray[i-w+1]+gray[i+w-1]+2*gray[i+w]+gray[i+w+1];
      const m=Math.hypot(gx,gy);mag[i]=m;if(m>25)values.push(m);
    }
    if(values.length<100)return null;
    values.sort((a,b)=>a-b);const threshold=Math.max(38,values[Math.floor(values.length*.78)]||38);
    const points=[];const margin=Math.max(3,Math.round(Math.min(w,h)*.012));
    for(let y=margin;y<h-margin;y+=1)for(let x=margin;x<w-margin;x+=1){const m=mag[y*w+x];if(m>=threshold)points.push({x,y,m})}
    if(points.length>18000){const stride=Math.ceil(points.length/18000);for(let i=points.length-1;i>=0;i--)if(i%stride)points.splice(i,1)}
    const angleStep=2,angles=90,diag=Math.ceil(Math.hypot(w,h)),rhoCount=diag*2+1;
    const cos=new Float32Array(angles),sin=new Float32Array(angles);
    for(let t=0;t<angles;t++){const a=t*angleStep*Math.PI/180;cos[t]=Math.cos(a);sin[t]=Math.sin(a)}
    const acc=new Uint16Array(angles*rhoCount);
    for(const p of points)for(let t=0;t<angles;t++){const r=Math.round(p.x*cos[t]+p.y*sin[t])+diag;const k=t*rhoCount+r;if(acc[k]<65535)acc[k]++}
    const lines=[];
    for(let t=0;t<angles;t++){
      const base=t*rhoCount;let local=[];
      for(let r=3;r<rhoCount-3;r++){const v=acc[base+r];if(v<Math.max(18,Math.min(w,h)*.045))continue;
        if(v>=acc[base+r-2]&&v>=acc[base+r-1]&&v>=acc[base+r+1]&&v>=acc[base+r+2])local.push({t,rho:r-diag,v});
      }
      local.sort((a,b)=>b.v-a.v);lines.push(...local.slice(0,3));
    }
    lines.sort((a,b)=>b.v-a.v);const top=lines.slice(0,42);if(top.length<4)return null;
    const pairs=[];
    for(let i=0;i<top.length;i++)for(let j=i+1;j<top.length;j++){
      const a=top[i],b=top[j],d=Math.abs(a.t-b.t),ad=Math.min(d,angles-d);
      if(ad>3)continue;
      const sep=Math.abs(a.rho-b.rho);if(sep<Math.min(w,h)*.12)continue;
      pairs.push({a,b,angle:(a.t+b.t)/2%angles,score:a.v+b.v});
    }
    const cross=(l1,l2)=>{
      const a=cos[l1.t],b=sin[l1.t],c=cos[l2.t],d=sin[l2.t],det=a*d-b*c;
      if(Math.abs(det)<.08)return null;
      return{x:(l1.rho*d-b*l2.rho)/det,y:(a*l2.rho-l1.rho*c)/det};
    };
    const inside=p=>p&&p.x>=0&&p.y>=0&&p.x<w&&p.y<h;
    let best=null,bestScore=0;
    for(let i=0;i<pairs.length;i++)for(let j=i+1;j<pairs.length;j++){
      const A=pairs[i],B=pairs[j],ad=Math.abs(A.angle-B.angle),ang=Math.min(ad,angles-ad);
      if(ang<18||ang>72)continue;
      const pts=[cross(A.a,B.a),cross(A.a,B.b),cross(A.b,B.b),cross(A.b,B.a)];
      if(!pts.every(inside))continue;
      const cx=pts.reduce((s,p)=>s+p.x,0)/4,cy=pts.reduce((s,p)=>s+p.y,0)/4;
      pts.sort((p,q)=>Math.atan2(p.y-cy,p.x-cx)-Math.atan2(q.y-cy,q.x-cx));
      const area=Math.abs(pts.reduce((s,p,k)=>{const q=pts[(k+1)%4];return s+p.x*q.y-q.x*p.y},0)/2);
      const fraction=area/(w*h);if(fraction<.055||fraction>.94)continue;
      const sides=pts.map((p,k)=>Math.hypot(p.x-pts[(k+1)%4].x,p.y-pts[(k+1)%4].y));
      const ratio1=Math.max(sides[0],sides[2])/Math.max(1,Math.min(sides[0],sides[2]));
      const ratio2=Math.max(sides[1],sides[3])/Math.max(1,Math.min(sides[1],sides[3]));
      if(ratio1>3.8||ratio2>3.8)continue;
      const borderPenalty=pts.reduce((s,p)=>s+(Math.min(p.x,p.y,w-p.x,h-p.y)<margin*1.5?1:0),0);
      const score=(A.score+B.score)*Math.sqrt(fraction)*(1-borderPenalty*.16);
      if(score>bestScore){bestScore=score;best=pts.map(p=>({x:p.x/scale,y:p.y/scale}))}
    }
    if(!best)return null;
    // Return TL, TR, BR, BL order.
    const cx=best.reduce((s,p)=>s+p.x,0)/4,cy=best.reduce((s,p)=>s+p.y,0)/4;
    best.sort((a,b)=>Math.atan2(a.y-cy,a.x-cx)-Math.atan2(b.y-cy,b.x-cx));
    let start=0;for(let i=1;i<4;i++)if(best[i].x+best[i].y<best[start].x+best[start].y)start=i;
    return best.slice(start).concat(best.slice(0,start));
  }
  window.SmartCrop={detect};
})();