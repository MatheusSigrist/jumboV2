#include "campo_page.hpp"

#include "../html_template.hpp"

namespace WebPages {

String renderCampoPage() {
  String body;
  body.reserve(2600);

  body += "<main class=\"panel\">";
  body += "<div class=\"header\">";
  body += "<div><div class=\"title\">Posicionamento em Campo</div><div class=\"muted\">Campo 2D vista superior em proporcao oficial</div></div>";
  body += HtmlTemplate::backButton("/");
  body += "</div>";

  body += "<div class=\"muted\" id=\"ultraMeta\" style=\"margin-bottom:10px;\">Referencias: campo 182 x 243 cm, circulo central diametro 60 cm, robo diametro 21 cm.</div>";

  body += "<div style=\"width:min(94vw,820px);margin:0 auto;\">";
  body += "<svg viewBox=\"0 0 1980 2590\" style=\"width:100%;height:auto;display:block;border-radius:12px;box-shadow:0 8px 20px rgba(0,0,0,.35);background:#0a0a0a;\" aria-label=\"Campo RoboCup 2D\">";

  body += "<rect x=\"40\" y=\"40\" width=\"1900\" height=\"2510\" rx=\"36\" fill=\"#0d0d0d\"/>";
  body += "<rect x=\"80\" y=\"80\" width=\"1820\" height=\"2430\" fill=\"#00b53f\"/>";

  body += "<rect x=\"80\" y=\"80\" width=\"1820\" height=\"2430\" fill=\"none\" stroke=\"#f4f4f4\" stroke-width=\"18\"/>";
  body += "<rect x=\"200\" y=\"200\" width=\"1580\" height=\"2190\" fill=\"none\" stroke=\"#f4f4f4\" stroke-width=\"10\" opacity=\"0.85\"/>";

  body += "<circle cx=\"990\" cy=\"1295\" r=\"300\" fill=\"none\" stroke=\"#f4f4f4\" stroke-width=\"12\"/>";

  body += "<path d=\"M 440 200 H 1540 V 300 Q 1540 450 1390 450 H 590 Q 440 450 440 300 Z\" fill=\"none\" stroke=\"#f4f4f4\" stroke-width=\"12\"/>";
  body += "<path d=\"M 590 2140 H 1390 Q 1540 2140 1540 2290 V 2390 H 440 V 2290 Q 440 2140 590 2140 Z\" fill=\"none\" stroke=\"#f4f4f4\" stroke-width=\"12\"/>";

  body += "<rect x=\"700\" y=\"110\" width=\"580\" height=\"60\" fill=\"#ffd21f\" opacity=\"0.95\"/>";
  body += "<rect x=\"700\" y=\"2420\" width=\"580\" height=\"60\" fill=\"#1f5eff\" opacity=\"0.95\"/>";

  body += "<g id=\"robot\" transform=\"translate(990 1540)\">";
  body += "<circle r=\"105\" fill=\"#202020\" stroke=\"#ffffff\" stroke-width=\"8\"/>";
  body += "<circle r=\"8\" fill=\"#ffffff\"/>";
  body += "<line x1=\"0\" y1=\"0\" x2=\"50\" y2=\"-40\" stroke=\"#ff6f3c\" stroke-width=\"9\" stroke-linecap=\"round\"/>";
  body += "</g>";

  body += "</svg>";
  body += "</div>";

  body += "</main>";

  String script;
  script.reserve(7600);
  script += "<script>";
  script += "const FIELD_W=182.0,FIELD_H=243.0,ROBOT_D=21.0,ROBOT_R=ROBOT_D/2.0;";
  script += "const COMP_TOL=22.0,MAX_JUMP=65.0;";
  script += "const TAU_FAST=0.26,TAU_SLOW=0.78;";
  script += "let lastX=FIELD_W/2.0,lastY=FIELD_H/2.0,lastConf=0.0;";
  script += "let lastTickMs=Date.now();";
  script += "const robotEl=document.getElementById('robot');const meta=document.getElementById('ultraMeta');";

  script += "function isValidDist(v){return Number.isFinite(v)&&v>1&&v<350;}";
  script += "function clamp(v,min,max){return Math.max(min,Math.min(max,v));}";
  script += "function mix(a,b,t){return a*(1-t)+b*t;}";

  script += "function complementary(prev,meas,conf,dtSec){";
  script += "const tau=(conf>=0.8)?TAU_FAST:TAU_SLOW;";
  script += "let alpha=Math.exp(-dtSec/Math.max(0.02,tau));";
  script += "alpha=clamp(alpha+(1-conf)*0.18,0.08,0.96);";
  script += "return alpha*prev+(1-alpha)*meas;}";

  script += "function estimateAxis(aRaw,bRaw,field,last,dtSec,axis){";
  script += "const hasA=isValidDist(aRaw),hasB=isValidDist(bRaw);";
  script += "const min=ROBOT_R,max=field-ROBOT_R,usable=field-2*ROBOT_R;";
  script += "let meas=last,conf=0.05,state='fallback',residual=999,sumAB=0;";

  script += "if(hasA&&hasB){";
  script += "const directA=clamp(aRaw+ROBOT_R,min,max);";
  script += "const directB=clamp(field-(bRaw+ROBOT_R),min,max);";
  script += "sumAB=Math.max(1.0,aRaw+bRaw);";
  script += "residual=Math.abs((aRaw+bRaw+2*ROBOT_R)-field);";
  script += "if(residual<=COMP_TOL){";
  script += "meas=(directA+directB)*0.5;conf=0.95;state='compativel';";
  script += "}else{";
  script += "const fracA=clamp(aRaw/sumAB,0,1);";
  script += "const fracB=clamp(bRaw/sumAB,0,1);";
  script += "const mapFromA=min+fracA*usable;";
  script += "const mapFromB=min+(1.0-fracB)*usable;";
  script += "meas=0.5*(mapFromA+mapFromB);";
  script += "conf=0.56;state='incompativel_mapeado';";
  script += "}";
  script += "}else if(hasA){";
  script += "meas=clamp(aRaw+ROBOT_R,min,max);conf=0.62;state=axis+'_single_a';";
  script += "}else if(hasB){";
  script += "meas=clamp(field-(bRaw+ROBOT_R),min,max);conf=0.62;state=axis+'_single_b';";
  script += "}";

  script += "const jump=meas-last;";
  script += "const limited=last+clamp(jump,-MAX_JUMP,MAX_JUMP);";
  script += "const filtered=complementary(last,limited,conf,dtSec);";
  script += "return {x:clamp(filtered,min,max),meas:clamp(meas,min,max),conf,state,residual,sum:sumAB};}";

  script += "function estimateFromUltras(u){";
  script += "const d=u.d,e=u.e,f=u.f,t=u.t;";
  script += "const now=Date.now();const dtSec=clamp((now-lastTickMs)/1000.0,0.05,0.8);lastTickMs=now;";

  script += "const rx=estimateAxis(e,d,FIELD_W,lastX,dtSec,'x');";
  script += "const ry=estimateAxis(f,t,FIELD_H,lastY,dtSec,'y');";

  script += "let x=rx.x;";
  script += "let y=ry.x;";

  script += "lastX=x;lastY=y;lastConf=(rx.conf+ry.conf)*0.5;";
  script += "return {x,y,conf:lastConf,xState:rx.state,yState:ry.state,xResidual:rx.residual,yResidual:ry.residual,xSum:rx.sum,ySum:ry.sum,xMeas:rx.meas,yMeas:ry.meas,dtSec};}";

  script += "function drawRobot(pos){";
  script += "const sx=80+pos.x*10.0;const sy=80+pos.y*10.0;";
  script += "robotEl.setAttribute('transform','translate('+sx.toFixed(1)+' '+sy.toFixed(1)+')');}";

  script += "function fmt(v){return Number.isFinite(v)?v.toFixed(1):'-';}";

  script += "async function tick(){";
  script += "try{";
  script += "const r=await fetch('/api/ultras',{cache:'no-store'});if(!r.ok)return;const j=await r.json();";
  script += "const u={d:Number(j.uD_x10)/10.0,e:Number(j.uE_x10)/10.0,f:Number(j.uF_x10)/10.0,t:Number(j.uT_x10)/10.0};";
  script += "const pos=estimateFromUltras(u);drawRobot(pos);";
  script += "meta.textContent='US (cm) D:'+fmt(u.d)+' E:'+fmt(u.e)+' F:'+fmt(u.f)+' T:'+fmt(u.t)+' | somaX:'+fmt(pos.xSum)+' somaY:'+fmt(pos.ySum)+' | meas x:'+fmt(pos.xMeas)+' y:'+fmt(pos.yMeas)+' | filt x:'+fmt(pos.x)+' y:'+fmt(pos.y)+' cm | conf:'+fmt(pos.conf*100)+'% | X:'+pos.xState+' Y:'+pos.yState+' | resid X:'+fmt(pos.xResidual)+' Y:'+fmt(pos.yResidual)+' | dt:'+fmt(pos.dtSec*1000)+' ms | idade:'+Number(j.idade_ms||0)+' ms';";
  script += "}catch(_){}}";

  script += "tick();setInterval(tick,250);";
  script += "</script>";

  return HtmlTemplate::wrapPage("Posicionamento em Campo", body, "", script);
}

}  // namespace WebPages
