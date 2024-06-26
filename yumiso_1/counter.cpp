#include "counter.h"


long flag_moneda;
long flag_premio;
long flag_bolsa;
long flag_stock;
long flag_games;

long costo;


// ----------------------------------------- coin_cointer
void IRAM_ATTR botonpress_monedas()
{
  mainRefresh = millis();
  flag_moneda++;
  obj["total"] = flag_moneda;
  flag_bolsa = (obj["total"].as<long>()) - (obj["init_bag"].as<long>());
  obj["bag"] = flag_bolsa;
  flag_games = flag_bolsa / costo;
  obj["games"] = flag_games;
  status_doc["status"] = "playing";
  //Serial.print("M");Serial.println(flag_moneda);
  saveConfig = true;
  send_log  = true;
  
  return;
}


// ----------------------------------------- gift_cointer
void IRAM_ATTR botonpress_premios()
{
  mainRefresh = millis();
  flag_premio++;
  obj["total_gift"] = flag_premio;
  flag_stock = (obj["t_gift"].as<long>()) - (obj["i_gift"].as<long>());
  obj["gift"] = flag_stock;
  status_doc["status"] = "gift";
  //Serial.print("P");Serial.println(flag_premio);
  saveConfig = true;
  send_log  = true;
  return;
}
