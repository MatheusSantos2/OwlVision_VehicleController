//----------------------------------------------------------------LIBRARIES-------------------------------------------------

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <RoboCore_Vespa.h>
#include <vector>
#include <array>

//Servidor TCP/IP
const char* ssid = "Apto 303";
const char* password = "P@$303W0rd";
int serverPort = 80;

WiFiServer server(serverPort);
WiFiClient client;

VespaMotors motorController;
std::vector<std::array<double, 2>> trajectoryPoints;

TaskHandle_t taskTrajectory;

int nextPointIndex = 0;
int numPoints = 0;
double currentX = 0; 
double currentY = 0;

double targetX = 0; 
double targetY = 0; 
double defaultSpeed = 80;
double distanceBetweenWheels = 12;
double tolerance = 1;
double theta_time = 1;

String lastTrajectoryData = "data"; 
String trajectoryData = "";
bool canGetNewTrajectory = true;

//-------------------Desligar Motores-------------------------------//
const unsigned long timeout = 90000;  // Tempo limite de 9 segundos
unsigned long lastMessageTime = 0;

//----------------------------------------------------------------SETUP-------------------------------------------------

void setup() {
  inicializeServer();
  xTaskCreatePinnedToCore(taskTrajectoryFunction, "TaskTrajectory", 10000, NULL, 1, &taskTrajectory, 1);
}

void inicializeServer(){
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.begin();
  Serial.print("Servidor TCP iniciado na porta: ");
  Serial.println(serverPort);
}

//---------------------------------------------------------------TASKS--------------------------------------------------

void taskTrajectoryFunction(void* parameter){
  while (true) {
    if (client.available()) {
      String data = client.readStringUntil('\n');

      String trajectoryLabel = data.substring(0, data.indexOf(';'));
      trajectoryLabel.trim();
      trajectoryData = removeFirstPosition(data);
      trajectoryData.trim();

      if (trajectoryLabel == "Trajectory" && lastTrajectoryData != trajectoryData && canGetNewTrajectory) 
      {
        Serial.println("Sucess - taskTrajectoryFunction - Label: " + trajectoryLabel + "; Data: " + trajectoryData);

        processTrajectoryData(trajectoryData);
        lastTrajectoryData = trajectoryData;
        resetTimer();
      }
      else{
        client.flush();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // Aguardar 10 milissegundos antes de verificar novamente
  }
}

String removeFirstPosition(String data){
  int index = 0;
  index = data.indexOf(';');
  if (index != -1){
    data = data.substring(index + 1);  // Extrai apenas os valores das coordenadas
  }
  return data;
}

void processTrajectoryData(String data){

  Serial.println("processTrajectoryData - Inicia o processamento da lista de posicoes");

  trajectoryPoints.clear();

  String point;
  int index = 0;

  while((index = data.indexOf(';')) != -1){
    point = data.substring(0, index);
    data = data.substring(index + 1);

    String xCoordinate = point.substring(0, point.indexOf(','));
    String yCoordinate = point.substring(point.indexOf(',') + 1);

    std::array<double, 2> newPoint = {xCoordinate.toDouble(), yCoordinate.toDouble()};

    trajectoryPoints.push_back(newPoint);
  }

  String xCoordinate = data.substring(0, data.indexOf(','));
  String yCoordinate = data.substring(data.indexOf(',') + 1);

  std::array<double, 2> newPoint = {xCoordinate.toDouble(), yCoordinate.toDouble()};

  trajectoryPoints.push_back(newPoint);

  printReceveidPoints();
}

void printReceveidPoints(){
  numPoints = 5;
  Serial.println("printReceveidPoints - Pontos de Trajetória Recebidos: ");
  
  for (int i = 0; i < numPoints; i++){
    Serial.print("Ponto ");
    Serial.print(i);
    Serial.print(": X=");
    Serial.print(trajectoryPoints[i][0]);
    Serial.print(", Y=");
    Serial.println(trajectoryPoints[i][1]);
  }
}

//----------------------------------------------------------------LOOP-------------------------------------------------

void loop(){
  connectTrajectoryServer();
  if(hasTimerExpired())
    enginePause();

  if (numPoints > 0){
    engineControl();
  }
  delay(100);
}

void connectTrajectoryServer(){
  if (server.hasClient())
  {
    if (!client.connected())
    {
      client = server.available();
      Serial.println("Nova conexão estabelecida com o cliente.");
    }else{
      server.stop();
      server.begin();
    }
  }
}

void engineControl(){
  
  double setpointX = readTargetPositionX();
  double setpointY = readTargetPositionY();
  
  if(setpointX != -150 && setpointY != -150)
  {
    // Calcular a diferença entre as coordenadas do ponto alvo e do robô
    double delta_x = setpointX - currentX;
    double delta_y = setpointY - currentY;

    // Calcular a orientação do ponto alvo em relação ao robô
    double theta_target = atan2(delta_x, delta_y);

    // Calcular as velocidades das rodas diretamente
    double leftSpeed = defaultSpeed + (distanceBetweenWheels / 2) * theta_target/theta_time;

    if (leftSpeed > 100){
      leftSpeed = 100;
    }

    double rightSpeed = defaultSpeed - (distanceBetweenWheels / 2) * theta_target/theta_time;
   
    if (rightSpeed > 100){
      rightSpeed = 100;
    }

    Serial.println("loop - Velocidade Direita: " + String(rightSpeed) + "; Velocidade Esquerda: " + String(leftSpeed) + " Angle: " + theta_target);
    accelerateMotors(rightSpeed, leftSpeed);
    delay(1000);

    Serial.println("loop - Pausa motores");
    motorController.stop();
    
    String message = String(leftSpeed) + "," + String(rightSpeed) + "," + String(setpointX) + "," + String(setpointY) + "," + String(nextPointIndex);
    sendMessage(message);

    currentX = setpointX; 
    currentY = setpointY;

    Serial.println("loop - spx: " + String(setpointX) + " ; spy: " + String(setpointY));
    updateNextPoint();
  }
}

void updateNextPoint(){
  if(numPoints > 0)
  {
    nextPointIndex = (nextPointIndex + 1) % numPoints;
    
    if(nextPointIndex == 4){
      trajectoryPoints.clear();
      motorController.stop();
      canGetNewTrajectory = true;
    }
  }
}

double readTargetPositionX(){
      if (!trajectoryPoints.empty()) {
        const auto& desiredPoint = trajectoryPoints[nextPointIndex];
        return desiredPoint[0];
    }
    return -150;
}

double readTargetPositionY(){
    if (!trajectoryPoints.empty()) {
        const auto& desiredPoint = trajectoryPoints[nextPointIndex];
        return desiredPoint[1];
    }
    return -150;
}

void accelerateMotors(int rightEngine, int leftEngine){
  //Serial.println("Acelerando motores - Motor Direito: " + String(rightEngine) + " Motor Esquerdo: " + String(leftEngine));
  motorController.turn(rightEngine, leftEngine);
}

void sendMessage(String message){
  Serial.println("sendMessage - " + message);
  client.println(message);
}

bool hasTimerExpired()
{
    return (millis() - lastMessageTime > timeout);
}

void resetTimer() {
  lastMessageTime = millis(); 
  canGetNewTrajectory = false; 
}

void enginePause(){
  Serial.println("enginePause - Pausa Motores após muito tempo sem receber uma nova mensagem");
  motorController.stop();
  trajectoryPoints.clear();
  nextPointIndex = 0;
  numPoints = 0;

  delay(1000);
}
