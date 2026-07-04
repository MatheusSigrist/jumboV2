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
  body += "<svg viewBox=\"0 0 1980 2590\" style=\"width:100%;height:auto;display:block;border-radius:12px;box-shadow:0 8px 20px rgba(0,0,0,.15);background:#0a0a0a;\" aria-label=\"Campo RoboCup 2D\">";

  body += "<rect x=\"40\" y=\"40\" width=\"1900\" height=\"2510\" rx=\"36\" fill=\"#0d0d0d\"/>";
  body += "<rect x=\"80\" y=\"80\" width=\"1820\" height=\"2430\" fill=\"#00b53f\"/>";

  body += "<rect x=\"80\" y=\"80\" width=\"1820\" height=\"2430\" fill=\"none\" stroke=\"#f4f4f4\" stroke-width=\"18\"/>";
  body += "<rect x=\"200\" y=\"200\" width=\"1580\" height=\"2190\" fill=\"none\" stroke=\"#f4f4f4\" stroke-width=\"10\" opacity=\"0.85\"/>";

  body += "<circle cx=\"990\" cy=\"1295\" r=\"300\" fill=\"none\" stroke=\"#f4f4f4\" stroke-width=\"12\"/>";

  body += "<path d=\"M 590 200 H 1390 Q 1540 200 1540 350 V 450 H 440 V 350 Q 440 200 590 200 Z\" fill=\"none\" stroke=\"#f4f4f4\" stroke-width=\"12\"/>";
  body += "<path d=\"M 440 2140 H 1540 V 2240 Q 1540 2390 1390 2390 H 590 Q 440 2390 440 2240 Z\" fill=\"none\" stroke=\"#f4f4f4\" stroke-width=\"12\"/>";

  body += "<rect x=\"700\" y=\"110\" width=\"580\" height=\"60\" fill=\"#1f5eff\" opacity=\"0.95\"/>";
  body += "<rect x=\"700\" y=\"2420\" width=\"580\" height=\"60\" fill=\"#ffd21f\" opacity=\"0.95\"/>";

  body += "<g id=\"robot\" transform=\"translate(990 1540)\">";
  body += "<circle r=\"105\" fill=\"#202020\" stroke=\"#ffffff\" stroke-width=\"8\"/>";
  body += "<circle r=\"8\" fill=\"#ffffff\"/>";
  body += "<line x1=\"0\" y1=\"0\" x2=\"50\" y2=\"-40\" stroke=\"#ff6f3c\" stroke-width=\"9\" stroke-linecap=\"round\"/>";
  body += "</g>";

  body += "</svg>";
  body += "</div>";

  body += "</main>";

  String script;
  script.reserve(5200);
  script += "<script>";
  script += "const FIELD_W=182.0,FIELD_H=243.0,ROBOT_D=21.0,ROBOT_R=ROBOT_D/2.0;";
  script += "const COMP_TOL=22.0,MAX_JUMP=65.0;";
  script += "let lastX=FIELD_W/2.0,lastY=FIELD_H/2.0,lastConf=0.0;";
  script += "const robotEl=document.getElementById('robot');const meta=document.getElementById('ultraMeta');";

  script += "function isValidDist(v){return Number.isFinite(v)&&v>1&&v<350;}";
  script += "function clamp(v,min,max){return Math.max(min,Math.min(max,v));}";

  script += "function pickAxis(cA,cB,residual,field,last){";
  script += "const hasA=isValidDist(cA),hasB=isValidDist(cB);";
  script += "const min=ROBOT_R,max=field-ROBOT_R;";
  script += "let x=last,conf=0.05,state='fallback';";

  script += "if(hasA&&hasB){";
  script += "const a=clamp(cA,min,max),b=clamp(cB,min,max);";
  script += "if(residual<=COMP_TOL){x=(a+b)*0.5;conf=0.95;state='compativel';}";
  script += "else{";
  script += "const score=(cand)=>{let s=1.0;s-=Math.min(1.0,Math.abs(cand-last)/MAX_JUMP);if(cand<=min||cand>=max)s-=0.25;return s;};";
  script += "const sa=score(a),sb=score(b);const best=sa>=sb?a:b;const other=sa>=sb?b:a;";
  script += "x=0.78*best+0.22*other;conf=0.45;state='incompativel_priorizado';}";
  script += "}";
  script += "else if(hasA){x=clamp(cA,min,max);conf=0.60;state='single_a';}";
  script += "else if(hasB){x=clamp(cB,min,max);conf=0.60;state='single_b';}";

  script += "return {x:clamp(x,min,max),conf,state};}";

  script += "function estimateFromUltras(u){";
  script += "const d=u.d,e=u.e,f=u.f,t=u.t;";
  script += "const xFromLeft=e+ROBOT_R;";
  script += "const xFromRight=FIELD_W-(d+ROBOT_R);";
  script += "const yFromTop=f+ROBOT_R;";
  script += "const yFromBottom=FIELD_H-(t+ROBOT_R);";

  script += "const xResidual=(isValidDist(d)&&isValidDist(e))?Math.abs((d+e+2*ROBOT_R)-FIELD_W):999;";
  script += "const yResidual=(isValidDist(f)&&isValidDist(t))?Math.abs((f+t+2*ROBOT_R)-FIELD_H):999;";

  script += "const rx=pickAxis(xFromLeft,xFromRight,xResidual,FIELD_W,lastX);";
  script += "const ry=pickAxis(yFromTop,yFromBottom,yResidual,FIELD_H,lastY);";

  script += "let x=0.72*lastX+0.28*rx.x;";
  script += "let y=0.72*lastY+0.28*ry.x;";
  script += "x=clamp(x,ROBOT_R,FIELD_W-ROBOT_R);";
  script += "y=clamp(y,ROBOT_R,FIELD_H-ROBOT_R);";

  script += "lastX=x;lastY=y;lastConf=(rx.conf+ry.conf)*0.5;";
  script += "return {x,y,conf:lastConf,xState:rx.state,yState:ry.state,xResidual,yResidual};}";

  script += "function drawRobot(pos){";
  script += "const sx=80+pos.x*10.0;const sy=80+pos.y*10.0;";
  script += "robotEl.setAttribute('transform','translate('+sx.toFixed(1)+' '+sy.toFixed(1)+')');}";

  script += "function fmt(v){return Number.isFinite(v)?v.toFixed(1):'-';}";

  script += "async function tick(){";
  script += "try{";
  script += "const r=await fetch('/api/ultras',{cache:'no-store'});if(!r.ok)return;const j=await r.json();";
  script += "const u={d:Number(j.uD_x10)/10.0,e:Number(j.uE_x10)/10.0,f:Number(j.uF_x10)/10.0,t:Number(j.uT_x10)/10.0};";
  script += "const pos=estimateFromUltras(u);drawRobot(pos);";
  script += "meta.textContent='US (cm) D:'+fmt(u.d)+' E:'+fmt(u.e)+' F:'+fmt(u.f)+' T:'+fmt(u.t)+' | x:'+fmt(pos.x)+' y:'+fmt(pos.y)+' cm | conf:'+fmt(pos.conf*100)+'% | X:'+pos.xState+' Y:'+pos.yState+' | resid X:'+fmt(pos.xResidual)+' Y:'+fmt(pos.yResidual)+' | idade:'+Number(j.idade_ms||0)+' ms';";
  script += "}catch(_){}}";

  script += "tick();setInterval(tick,250);";
  script += "</script>";

  return HtmlTemplate::wrapPage("Posicionamento em Campo", body, "", script);
}

}  // namespace WebPages
