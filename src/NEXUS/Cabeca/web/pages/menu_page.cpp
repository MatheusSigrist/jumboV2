#include "menu_page.hpp"

#include "../html_template.hpp"

namespace WebPages {

String renderMenuPage() {
  String body;
  body.reserve(1300);

  body += "<main class=\"panel\">";
  body += "<div class=\"header\">";
  body += "<div class=\"title\">NEXUS Cabeca - Menu Principal</div>";
  body += "<div class=\"muted\">4 opcoes organizadas em paginas separadas</div>";
  body += "</div>";

  body += "<section class=\"grid\">";
  body += "<a class=\"card\" href=\"/mapa\"><h3>Mapa de Sensores</h3><p>Visualizacao dos 32 sensores da placa Pe.</p></a>";
  body += "<a class=\"card\" href=\"/campo\"><h3>Posicionamento em Campo</h3><p>Tela reservada para o modulo de campo.</p></a>";
  body += "<a class=\"card\" href=\"/bussola\"><h3>Bussola</h3><p>Radar 0-360 com atual, referencia e erro angular.</p></a>";
  body += "<a class=\"card\" href=\"/extra\"><h3>Opcao 4</h3><p>Espaco reservado para a proxima funcionalidade.</p></a>";
  body += "</section>";

  body += "</main>";

  return HtmlTemplate::wrapPage("NEXUS Cabeca - Menu", body);
}

}  // namespace WebPages
