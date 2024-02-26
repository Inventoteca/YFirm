#include "counter.h"


// ----------------------------------------- coin_cointer
void IRAM_ATTR botonpress_monedas() {
  flag_moneda++;
  obj["total"] = flag_moneda;
  flag_bolsa = (obj["total"].as<long>()) - (obj["init_bag"].as<long>());
  doc_status["bolsa"] = flag_bolsa;
  doc_status["total"] = obj["total"];
  doc_status["init_bag"] = obj["init_bag"];
  //obj["bag"] = flag_bolsa;
  //Serial.print("M");Serial.println(flag_moneda);
  saveConfig = true;
  return;
}


// ----------------------------------------- gift_cointer
void IRAM_ATTR botonpress_premios() {
  flag_premio++;
  obj["gift"] = flag_premio;
  doc_status["gift"] = obj["gift"];
  //Serial.print("P");Serial.println(flag_premio);
  saveConfig = true;
  return;
}
