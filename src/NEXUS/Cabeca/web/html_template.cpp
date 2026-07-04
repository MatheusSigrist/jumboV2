#include "html_template.hpp"

namespace HtmlTemplate {

String wrapPage(const char* title,
                const String& bodyHtml,
                const String& extraHead,
                const String& extraScript) {
  String html;
  html.reserve(1200 + bodyHtml.length() + extraHead.length() + extraScript.length());

  html += "<!doctype html><html lang=\"pt-BR\"><head>";
  html += "<meta charset=\"utf-8\"/>";
  html += "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\"/>";
  html += "<title>";
  html += title;
  html += "</title>";

  html += "<style>";
  html += ":root{--bg1:#f7f1e3;--bg2:#e8dcc4;--card:#fff8ea;--ink:#2f2718;--muted:#6f634d;--line:#d7ccb5;--btn:#1f7a5b;--btn2:#145a43;}";
  html += "*{box-sizing:border-box;}";
  html += "body{margin:0;min-height:100vh;padding:16px;display:grid;place-items:center;font-family:'Segoe UI',Tahoma,sans-serif;color:var(--ink);background:linear-gradient(160deg,var(--bg1),var(--bg2));}";
  html += ".panel{width:min(980px,100%);background:var(--card);border:1px solid var(--line);border-radius:16px;box-shadow:0 12px 28px rgba(67,49,18,.15);padding:18px;}";
  html += ".header{display:flex;justify-content:space-between;align-items:center;gap:12px;flex-wrap:wrap;margin-bottom:12px;}";
  html += ".title{font-size:1.2rem;font-weight:700;}";
  html += ".muted{color:var(--muted);font-size:.92rem;}";
  html += ".btn{display:inline-block;border:0;border-radius:10px;padding:10px 14px;text-decoration:none;font-weight:700;color:#fff;background:linear-gradient(180deg,var(--btn),var(--btn2));}";
  html += ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(190px,1fr));gap:12px;}";
  html += ".card{display:block;text-decoration:none;color:inherit;border:1px solid var(--line);border-radius:12px;padding:14px;background:#fffdf7;}";
  html += ".card h3{margin:0 0 6px;font-size:1rem;}";
  html += ".card p{margin:0;color:var(--muted);font-size:.9rem;}";
  html += extraHead;
  html += "</style></head><body>";
  html += bodyHtml;
  html += extraScript;
  html += "</body></html>";

  return html;
}

String backButton(const char* href) {
  String b;
  b.reserve(64);
  b += "<a class=\"btn\" href=\"";
  b += href;
  b += "\">Voltar</a>";
  return b;
}

}  // namespace HtmlTemplate
