#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// ---------- WIFI ----------
const char* WIFI_SSID = "Iphone";
const char* WIFI_PASSWORD = "0760853136";

// ---------- AWS IOT ----------
const char* AWS_ENDPOINT = "a2mlrmhn7n3wcj-ats.iot.eu-north-1.amazonaws.com"; // xxxxx-ats.iot.<region>.amazonaws.com
const int AWS_PORT = 8883;
const char* MQTT_TOPIC = "test/esp32/data";

// ---------- AMAZON ROOT CA 1 ----------
const char AWS_ROOT_CA[] = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";

// ---------- DEVICE CERT ----------
const char AWS_CERT[] = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDWTCCAkGgAwIBAgIURh9bRfXbX9M0d79LMIo1btdtJSEwDQYJKoZIhvcNAQEL
BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g
SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTI2MDEwMzIzNDA0
N1oXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0
ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAK0DwcouG1MXc61UrVzn
EEM8BsZ6dGmCmfkWF+iryHWEeoiTp00KVGN9FUds2BB7S+4gN2VwGrTvmNA0V0L9
cqbshmPE9ycvs28qKY73//TEfGn8VhTBroR2Q516uR0MDyAH8ICrqMrkNISJJxwI
z13qsTN7PS8EtGYFGMOlpYM2jdVux5d7i5q1Q/MiHtyHsmkcEQsUYwlaIeONj8eC
v5rYCrZSUduDYDiKRRKEFyLssExIyq903HdoEpb8/JnjbBkXRp/yQbme00jASHuZ
icdC/hknWQc7frEtUnZw8IcCsmiVQ/d7h902aXHzXCyXUWsYRsxlMiiaV2Yk9EwZ
fuMCAwEAAaNgMF4wHwYDVR0jBBgwFoAUTM5/QHDAKImgN3UDLMQ3d2yW7AswHQYD
VR0OBBYEFPN7WKxBYLypvpaS5wwbJhs9UJYDMAwGA1UdEwEB/wQCMAAwDgYDVR0P
AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQA3Orj4fGnqEgm/udrdGEPeLZEu
MZjfZCw9yhaDEqm3aK8vO7M1FCFTmtVak5MBdJgbY475Xrdy5zX1gxIevTGUp632
YWP+UdI5w/8GBB7P9jB15eI+sqyK1zxeuhqkYVwgJl5Cut6JshUCE3C6QU2ATRlD
7j2y39YEjY1QLA4gqrjonz6JRg1Ng42utRcF374UasYbsY6vTzupIUC9mh8VLfv7
O6NQLw2levh/ylmUfmZIIqccWGBfmScW9cgzXy/RVqzE6bgDgTh/A8hSqi6bYzs6
1M0MTuS4jZQnOgU5wZ0z965sZQKWV9jO080ftuyAJYUfBpYPG9cEHicHxQz4
-----END CERTIFICATE-----
)EOF";

// ---------- PRIVATE KEY ----------
const char AWS_PRIVATE_KEY[] = R"EOF(
-----BEGIN RSA PRIVATE KEY-----
MIIEowIBAAKCAQEArQPByi4bUxdzrVStXOcQQzwGxnp0aYKZ+RYX6KvIdYR6iJOn
TQpUY30VR2zYEHtL7iA3ZXAatO+Y0DRXQv1ypuyGY8T3Jy+zbyopjvf/9MR8afxW
FMGuhHZDnXq5HQwPIAfwgKuoyuQ0hIknHAjPXeqxM3s9LwS0ZgUYw6WlgzaN1W7H
l3uLmrVD8yIe3IeyaRwRCxRjCVoh442Px4K/mtgKtlJR24NgOIpFEoQXIuywTEjK
r3Tcd2gSlvz8meNsGRdGn/JBuZ7TSMBIe5mJx0L+GSdZBzt+sS1SdnDwhwKyaJVD
93uH3TZpcfNcLJdRaxhGzGUyKJpXZiT0TBl+4wIDAQABAoIBACFf0igQENFMetH6
vZ5oLCjsEVqEEKSxvsXfzyjgykBxF7n00Zb44SJ35HzssBktz43VxRnaMCyq801m
a1bO0SkGAz6Hpi129CQDfBTKHiO3K1C+jlResC15Kr3cCI1j4B8LqQcJFfxdXQwb
8OFliarSNPB7W2gQfkQXw5kqAO14bU1001YAr/yM9xayeXYVFBr/XdtcZBeKtkAq
/n8Nkh114X3MLnzox54CZU3N3Opjqc1jx7AOmvEvJjtLj8sq3K6qUTOFyeGwAw7V
NoVzZX9DflLz3sEpNH1KhFFZLV1F+HCg9D6r2og7MFrrdSyJr3lXDSq0w9WUwPRN
vACKHIECgYEA2v6SLxktXXeJugfsrPIHDFrQUQw1Yh8DVHInqoyZsC4yxbCpE0mU
tlDo2nOZLam7/UCoBHnvyomsNIKRb9EnMEsgXr352WbeY81WJrY+Ah0Sqsj3p+3n
KQaf5CsXkGH/1sGeG9CBG28X61AX5jjT7CqACdPrNLzgXksDwQSYCSECgYEAykAo
rdHv7+eI8FmUdGAQGHdxwxlvHQYQs4uyoRu5XkHBmaxn6YnjO2ew5r+9S2WoVRB0
uqEfRgQMJnGyFCkcoGAbw2+eeCD6vYIhz+g6yWxdpzhT8fV84dOO+k4nYlzqHk2N
zIIumoRwshdpffild0EHj+tbHjf/A7xtSyDTc4MCgYBMML+JVVgcKaeoKnOkY/wh
x0Ksv/OetK2C5sh4JLyfuCL/9ouMY1Ay8glhX1COu3vlC2apUAcmTymzhy20Wm8o
9SpI7A2OHqUG0fzEMSl3sMe61XqcWT/QXTapunhTSlUpUWmBwdP5SHho7Q+zkFfi
1ZKAWNN/IKtrAuxGbiO7IQKBgDVzT0zgtrCIWEBs7Db1TEurBX2yMxNQjwlkWgkJ
8qteZXPfhHbL4inI9Y+GDNjoPNx+RNstyb4PQ8bFNXLuioo33B6CWTcWQC3lPlpb
3W1uHjIbSNQhNKfZ6WdtUCtGsvjfNiJeJULgzYfDeDW6iMBDh2QZpzMNSXALVDcO
rdNRAoGBAIZmwtonMBBABbT5CbSdt2Doro3Ygrwj3G7CDYQE8oepJfHAEJJ0QIht
rxW6j2nIIX0U0MwqOp8++ZyZcPUSyHZENmAECfMDPHO7/IILcdBIyTHiw1ybcJI5
atvXilM+hsDBIDcao3tZ7+7WRXJnWRnDrG/RoJRcmJ/12Mhp5XFW
-----END RSA PRIVATE KEY-----
)EOF";

// ---------- MQTT ----------
WiFiClientSecure net;
PubSubClient client(net);

void connectAWS() {
  while (!client.connected()) {
    Serial.print("Connecting to AWS IoT...");
    if (client.connect("esp32-main-node")) {
      Serial.println("CONNECTED");
    } else {
      Serial.print("FAILED, rc=");
      Serial.println(client.state());
      delay(3000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  // TLS certs
  net.setCACert(AWS_ROOT_CA);
  net.setCertificate(AWS_CERT);
  net.setPrivateKey(AWS_PRIVATE_KEY);

  // MQTT
  client.setServer(AWS_ENDPOINT, AWS_PORT);
  connectAWS();

  // Publish test message
  client.publish(MQTT_TOPIC,
    "{ \"device\": \"esp32-dev\", \"status\": \"hello aws iot\" }");
}

void loop() {
  client.loop();
}
