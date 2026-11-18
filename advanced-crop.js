/* SMART SCAN PRO - Multi-engine document detector
 * OpenCV pipeline: Canny, adaptive threshold, contours, Hough, Harris, FAST, ORB.
 * Perspective correction is performed by the scanner's existing homography renderer.
 */
(function(){
'use strict';
let cvPromise=null;
function loadOpenCV(){
  if(window.cv && window.cv.Mat) return Promise.resolve(window.cv);
  if(cvPromise) return cvPromise;
  cvPromise=new Promise((resolve,reject)=>{
    const timeout=setTimeout(()=>reject(new Error('انتهت مهلة تحميل OpenCV')),30000);
    const finish=()=>{if(window.cv&&window.cv.Mat){clearTimeout(timeout);resolve(window.cv);return false}return true};
    if(!document.querySelector('script[data-smart-opencv]')){
      const s=document.createElement('script');s.src='https://docs.opencv.org/4.x/opencv.js';s.async=true;s.dataset.smartOpencv='1';
      s.onerror=()=>{clearTimeout(timeout);reject(new Error('تعذر تحميل OpenCV'))};
      s.onload=()=>{const started=Date.now();(function wait(){if(!finish()){return}if(Date.now()-started>29000){clearTimeout(timeout);reject(new Error('OpenCV لم يكتمل تشغيله'));return}setTimeout(wait,100)})()};
      document.head.appendChild(s);
    } else {
      const started=Date.now();(function wait(){if(!finish())return;if(Date.now()-started>29000){clearTimeout(timeout);reject(new Error('OpenCV لم يكتمل تشغيله'));return}setTimeout(wait,100)})();
    }
  });
  return cvPromise;
}
function order(p){
  const cx=p.reduce((s,v)=>s+v.x,0)/4,cy=p.reduce((s,v)=>s+v.y,0)/4;
  const q=p.slice().sort((a,b)=>Math.atan2(a.y-cy,a.x-cx)-Math.atan2(b.y-cy,b.x-cx));
  let k=0;for(let i=1;i<4;i++)if(q[i].x+q[i].y<q[k].x+q[k].y)k=i;
  return q.slice(k).concat(q.slice(0,k));
}
function area(p){let a=0;for(let i=0;i<4;i++){const q=p[i],r=p[(i+1)%4];a+=q.x*r.y-r.x*q.y}return Math.abs(a/2)}
function sideRatio(p){const s=p.map((v,i)=>Math.hypot(v.x-p[(i+1)%4].x,v.y-p[(i+1)%4].y));return Math.max(s[0],s[2])/Math.max(1,Math.min(s[0],s[2]))<4.2&&Math.max(s[1],s[3])/Math.max(1,Math.min(s[1],s[3]))<4.2}
function detectWithOpenCV(img,cv){
  const maxSide=1100,scale=Math.min(1,maxSide/Math.max(img.naturalWidth,img.naturalHeight));
  const w=Math.max(2,Math.round(img.naturalWidth*scale)),h=Math.max(2,Math.round(img.naturalHeight*scale));
  const canvas=document.createElement('canvas');canvas.width=w;canvas.height=h;canvas.getContext('2d').drawImage(img,0,0,w,h);
  const src=cv.imread(canvas),gray=new cv.Mat(),blur=new cv.Mat(),edges=new cv.Mat(),kernel=cv.Mat.ones(3,3,cv.CV_8U);
  let best=null,bestScore=0,contourRuns=0,houghCount=0,harrisCount=0,fastCount=0,orbCount=0,fastRan=false,orbRan=false;
  const featurePoints=[],houghLines=[];
  const candidates=[];
  try{
    cv.cvtColor(src,gray,cv.COLOR_RGBA2GRAY);cv.GaussianBlur(gray,blur,new cv.Size(5,5),0,0,cv.BORDER_DEFAULT);
    const edgeSets=[];
    for(const pair of [[25,80],[40,120],[60,180],[15,50]]){
      const e=new cv.Mat();cv.Canny(blur,e,pair[0],pair[1]);cv.morphologyEx(e,e,cv.MORPH_CLOSE,kernel);edgeSets.push(e);
    }
    const adaptive=new cv.Mat();cv.adaptiveThreshold(blur,adaptive,255,cv.ADAPTIVE_THRESH_GAUSSIAN_C,cv.THRESH_BINARY,31,11);
    const adaptiveInv=new cv.Mat();cv.adaptiveThreshold(blur,adaptiveInv,255,cv.ADAPTIVE_THRESH_GAUSSIAN_C,cv.THRESH_BINARY_INV,31,11);
    const masks=edgeSets.concat([adaptive,adaptiveInv]);
    for(const mask of masks){
      const contours=new cv.MatVector(),hierarchy=new cv.Mat();
      cv.findContours(mask,contours,hierarchy,cv.RETR_LIST,cv.CHAIN_APPROX_SIMPLE);
      const ranked=[];
      for(let i=0;i<contours.size();i++){const cnt=contours.get(i),a=Math.abs(cv.contourArea(cnt));if(a>w*h*.035&&a<w*h*.985)ranked.push({cnt,a})}
      ranked.sort((a,b)=>b.a-a.a);
      for(const item of ranked.slice(0,45)){
        const perimeter=cv.arcLength(item.cnt,true);
        for(const eps of [.012,.018,.025,.035,.05]){
          const approx=new cv.Mat();cv.approxPolyDP(item.cnt,approx,eps*perimeter,true);
          if(approx.rows===4&&cv.isContourConvex(approx)){
            const data=approx.data32S,pts=[];
            for(let j=0;j<8;j+=2)pts.push({x:data[j],y:data[j+1]});
            const q=order(pts),fraction=area(q)/(w*h);
            if(fraction>.035&&fraction<.985&&sideRatio(q)){
              const shapeArea=Math.abs(cv.contourArea(approx));
              candidates.push({p:q,base:fraction,shape:shapeArea/Math.max(1,area(q))});
            }
          }
          approx.delete();
        }
      }
      contourRuns++;contours.delete();hierarchy.delete();
    }
    // Hough line transform supplies independent line evidence.
    const hough=new cv.Mat();cv.HoughLines(edgeSets[1],hough,1,Math.PI/180,Math.max(35,Math.round(Math.min(w,h)*.12)));
    houghCount=hough.rows;;
    for(let i=0;i<Math.min(hough.rows,500);i++){const rho=hough.data32F[i*2],theta=hough.data32F[i*2+1];houghLines.push({rho,theta})};
    hough.delete();
    // Harris corner response and Shi-Tomasi/Harris feature points.
    const harris=new cv.Mat();cv.cornerHarris(blur,harris,2,3,.04);
    const corners=new cv.Mat(),featureMask=new cv.Mat();
    cv.goodFeaturesToTrack(blur,corners,120,.01,8,featureMask,3,true,.04);
    harrisCount=corners.rows;for(let i=0;i<corners.rows;i++)featurePoints.push({x:corners.data32F[i*2],y:corners.data32F[i*2+1]});
    harris.delete();corners.delete();featureMask.delete();
    // FAST and ORB keypoints add independent corner/feature evidence.
    try{if(cv.FastFeatureDetector){const f=cv.FastFeatureDetector.create(20,true);const kp=new cv.KeyPointVector();f.detect(gray,kp);fastRan=true;fastCount=kp.size();for(let i=0;i<Math.min(kp.size(),1000);i++){const z=kp.get(i).pt;featurePoints.push({x:z.x,y:z.y})}kp.delete();f.delete()}}catch(e){}
    try{if(cv.ORB){const o=cv.ORB.create(300);const kp=new cv.KeyPointVector();o.detect(gray,kp);orbRan=true;orbCount=kp.size();for(let i=0;i<Math.min(kp.size(),1000);i++){const z=kp.get(i).pt;featurePoints.push({x:z.x,y:z.y})}kp.delete();o.delete()}}catch(e){}
    // Rank contour candidates by size and geometric completeness; prefer outer boundary.
    candidates.sort((a,b)=>b.base-a.base);
    for(const cand of candidates){
      const q=cand.p;
      const edge= new cv.Mat();cv.Canny(blur,edge,35,110);
      let support=0;
      for(let k=0;k<4;k++){
        const a=q[k],b=q[(k+1)%4],n=Math.max(20,Math.min(100,Math.round(Math.hypot(b.x-a.x,b.y-a.y)/6)));
        let hits=0;for(let j=0;j<n;j++){const f=(j+.5)/n,x=Math.round(a.x+(b.x-a.x)*f),y=Math.round(a.y+(b.y-a.y)*f);let found=false;for(let oy=-2;oy<=2&&!found;oy++)for(let ox=-2;ox<=2;ox++){const xx=x+ox,yy=y+oy;if(xx>=0&&yy>=0&&xx<w&&yy<h&&edge.ucharPtr(yy,xx)[0]>0){found=true;break}}if(found)hits++}
        support+=hits/n;
      }
      edge.delete();support/=4;
      let houghSupport=0,featureSupport=0;
      for(let k=0;k<4;k++){
        const a=q[k],b=q[(k+1)%4],dx=b.x-a.x,dy=b.y-a.y;
        const theta=(Math.atan2(dy,dx)+Math.PI/2+Math.PI*2)%(Math.PI*2);
        const rho=a.x*Math.cos(theta)+a.y*Math.sin(theta);
        if(houghLines.some(line=>{
          let da=Math.abs(line.theta-theta);da=Math.min(da,Math.abs(Math.PI-da),Math.abs(2*Math.PI-da));
          return da<.16&&Math.abs(line.rho-rho)<Math.max(7,Math.min(w,h)*.025);
        }))houghSupport++;
        const radius=Math.max(10,Math.min(w,h)*.035);
        if(featurePoints.some(pt=>Math.hypot(pt.x-a.x,pt.y-a.y)<radius))featureSupport++;
      }
      const score=Math.pow(cand.base,.72)*(.35+support*1.7)*(.7+cand.shape*.3)*
        (1+houghSupport*.035)*(1+featureSupport*.025);
      if(support>.10&&score>bestScore){bestScore=score;best=q.map(p=>({x:p.x/scale,y:p.y/scale}))}
    }
    masks.forEach(m=>m.delete());kernel.delete();
  }finally{src.delete();gray.delete();blur.delete();edges.delete()}
  if(!best)return null;
  const engines=['OpenCV','Canny','Adaptive Threshold','Contour Detection','Hough Lines','Harris'];if(fastRan)engines.push('FAST');if(orbRan)engines.push('ORB');engines.push('Perspective Transform');\n  return {corners:best,engines,stats:{contourRuns,houghCount,harrisCount,fastCount,orbCount}};
}
async function detect(img){
  try{return detectWithOpenCV(img,await loadOpenCV())}
  catch(error){
    if(window.SmartCrop&&window.SmartCrop.detect){const p=window.SmartCrop.detect(img);if(p)return {corners:p,engines:['Offline Sobel/Hough fallback'],warning:error.message}}
    throw error;
  }
}
window.AdvancedSmartCrop={detect,loadOpenCV};
})();