#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "ESP32-Robot";
const char* password = "12345678";

WebServer server(80);

// 固定热点IP
IPAddress local_ip(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

void sendCmd(char c)
{
  Serial2.write(c);
  Serial.println(c);
  server.send(200,"text/plain",String(c));
}

void handleRoot()
{
String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">

<style>
body {text-align:center;font-family:Arial;}
button {width:120px;height:60px;font-size:20px;margin:10px;}
</style>

</head>

<body>

<h2>ESP32 Robot Control</h2>

<button onclick="send('F')">Forward</button><br>

<button onclick="send('L')">Left</button>
<button onclick="send('S')">Stop</button>
<button onclick="send('R')">Right</button><br>

<button onclick="send('B')">Back</button>

<script>
function send(c)
{
fetch("/cmd?c="+c);
}
</script>

</body>
</html>
)rawliteral";

server.send(200,"text/html",html);
}

void handleCmd()
{
  if (!server.hasArg("c") || server.arg("c").length() == 0)
  {
    server.send(400, "text/plain", "Bad Request");
    return;
  }

  char c = server.arg("c")[0];
  sendCmd(c);
}

void setup()
{
  Serial.begin(115200);

  // ESP32 UART2
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  // ===== 只改进 WiFi 连接 =====
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);   // 关闭省电，提高热点响应稳定性

  // 配置AP固定IP
  if (!WiFi.softAPConfig(local_ip, gateway, subnet))
  {
    Serial.println("AP Config Failed");
  }

  // 启动热点：SSID, 密码, 信道, 是否隐藏, 最大连接数
  bool ap_ok = WiFi.softAP(ssid, password, 6, 0, 4);

  if (ap_ok)
  {
    Serial.println("WiFi AP started");
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
  }
  else
  {
    Serial.println("WiFi AP start failed");
  }
  // ===== WiFi 改进结束 =====

  server.on("/", handleRoot);
  server.on("/cmd", handleCmd);

  server.begin();
}

void loop()
{
  server.handleClient();
}