#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <RoboCore_Vespa.h>
#include <PID_v1.h>

#include <vector>
#include <array>

const char* ssid = "Apto 303";
const char* password = "P@$303W0rd";
int serverPort = 80;

WiFiServer server(serverPort);
WiFiClient client;
VespaMotors motorController;

double targetX = 0; // Coordenada x desejada
double targetY = 0; // Coordenada y desejada
double currentX = 0; // Coordenada x atual
double currentY = 0; // Coordenada y atual
double targetSpeed = 0; // Velocidade desejada
double currentSpeed = 0; // Velocidade atual
int nextPointIndex = 0;
int numPoints = 0;

unsigned long previousTime = 0;
unsigned long currentTime = 0;
double previousX = 0;
double previousY = 0;

std::vector<std::array<double, 2>> trajectoryPoints;

class PIDController {
private:
  double* input;
  double* output;
  double* setpoint;
  double Kp;
  double Ki;
  double Kd;
  int direction;

  PID pid;

public:
  PIDController(double* input, double* setpoint, double* output, double Kp, double Ki, double Kd, int direction)
    : input(input), output(output), setpoint(setpoint), Kp(Kp), Ki(Ki), Kd(Kd), direction(direction), pid(PID(input, output, setpoint, Kp, Ki, Kd, direction))
  {
  }

  void compute()
  {
    pid.Compute();
  }

  void setTunings(double Kp, double Ki, double Kd)
  {
    pid.SetTunings(Kp, Ki, Kd);
  }

  void setOutputLimits(double min, double max)
  {
    pid.SetOutputLimits(min, max);
  }

  void setMode(int mode)
  {
    pid.SetMode(mode);
  }
};

// Controladores PI
PIDController posXController(&currentX, &targetX, &currentSpeed, 2, 5, 0, DIRECT);
PIDController posYController(&currentY, &targetY, &currentSpeed, 2, 5, 0, DIRECT);
PIDController speedController(&currentSpeed, &targetSpeed, &currentSpeed, 2, 5, 0, DIRECT);


void setup() 
{
  inicializeServer();
  inicializePIDController();
}

void loop() 
{
  connectServer();
  
  updateCurrentPosition();

   if (reachedNextPoint()) {
    // Atualiza o próximo ponto da trajetória
    updateNextPoint();
  }

  processCurrentSpeed();
  // Executa os controladores PI de posição
  posXController.compute();
  posYController.compute();

  // Calcula a velocidade desejada a partir das coordenadas x e y desejadas
  targetSpeed = sqrt(pow(targetX - currentX, 2) + pow(targetY - currentY, 2));

  // Executa o controlador PI de velocidade
  speedController.compute();

  // Mapeia os valores de velocidade e direção para o intervalo de -100 a 100
  int mappedSpeed = map(currentSpeed, -100, 100, -255, 255);
  int mappedDirection = currentSpeed > 0 ? VespaMotors::FORWARD : VespaMotors::BACKWARD;

  // Atualiza os motores
  motorController.setSpeedLeft(mappedSpeed);
  motorController.setSpeedRight(mappedSpeed);
  motorController.forward(mappedDirection);

  // Aguarda um intervalo de tempo antes de repetir o loop
  delay(1000);
}

void inicializeServer()
{
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

void inicializePIDController()
{
  posXController.setMode(AUTOMATIC);
  posYController.setMode(AUTOMATIC);
  speedController.setMode(AUTOMATIC);
}

void connectServer()
{
  // Verifica se há novas conexões de clientes
  if (server.hasClient()) 
  {
    // Se há um cliente novo, desconecta o cliente anterior
    if (client.connected()) 
    {
      client.stop();
    }
    // Aceita a nova conexão
    client = server.available();
    Serial.println("Novo cliente conectado!");
  }

  // Verifica se há dados disponíveis para leitura no cliente TCP
  if (client.available()) {
    // Lê os dados recebidos
    String data = client.readStringUntil('\n');
    // Processa os dados recebidos
    processTrajectoryData(data);
  }
}

void processTrajectoryData(String data) {
  // Limpa o vetor de trajetória antes de receber os novos pontos
  trajectoryPoints.clear();

  // Extrai os pontos de trajetória da string recebida
  String point;
  int index = 0;

  while ((index = data.indexOf(',')) != -1) {
    point = data.substring(0, index);
    data = data.substring(index + 1);

    std::array<double, 2> newPoint;
    newPoint[0] = point.toDouble();
    
    point = data.substring(0, data.indexOf(','));
    data = data.substring(data.indexOf(',') + 1);
    
    newPoint[1] = point.toDouble();

    // Adiciona o novo ponto ao vetor de trajetória
    trajectoryPoints.push_back(newPoint);
  }

  numPoints = trajectoryPoints.size();
  // Imprime os pontos de trajetória recebidos
  Serial.println("Pontos de Trajetória Recebidos:");
  for (int i = 0; i < trajectoryPoints.size(); i++) {
    Serial.print("Ponto ");
    Serial.print(i);
    Serial.print(": X=");
    Serial.print(trajectoryPoints[i][0]);
    Serial.print(", Y=");
    Serial.println(trajectoryPoints[i][1]);
  }
}

// Função para atualizar a posição atual
void updateCurrentPosition() {
  // Verifica se a lista de pontos da trajetória está vazia
  if (numPoints == 0) {
    // Se estiver vazia, defina as coordenadas atuais como 0
    currentX = 0;
    currentY = 0;
    return;
  }

  // Verifica se o índice do próximo ponto está fora dos limites da lista de pontos
  if (nextPointIndex >= numPoints) {
    // Se estiver fora dos limites, define as coordenadas atuais como as coordenadas do último ponto
    currentX = trajectoryPoints[numPoints - 1][0];
    currentY = trajectoryPoints[numPoints - 1][1];
    return;
  }

  // Obtém as coordenadas atuais da lista de pontos
  currentX = trajectoryPoints[nextPointIndex][0];
  currentY = trajectoryPoints[nextPointIndex][1];
}

// Função para verificar se alcançou o próximo ponto da trajetória
bool reachedNextPoint() {
  // Verifica se a distância até o próximo ponto é menor que uma tolerância
  double distance = sqrt(pow(targetX - currentX, 2) + pow(targetY - currentY, 2));
  return distance < 0.1; // Defina a tolerância adequada
}

// Função para atualizar o próximo ponto da trajetória
void updateNextPoint() {
  // Verifica se a lista de pontos da trajetória está vazia
  if (numPoints == 0) {
    // Se estiver vazia, defina os valores de targetX e targetY como 0
    targetX = 0;
    targetY = 0;
    return;
  }

  // Incrementa o índice do próximo ponto
  nextPointIndex = (nextPointIndex + 1) % numPoints;

  // Obtém as coordenadas x e y do próximo ponto da trajetória
  targetX = trajectoryPoints[nextPointIndex][0];
  targetY = trajectoryPoints[nextPointIndex][1];
}

void processCurrentSpeed()
{
  // Calcula a velocidade com base na variação das coordenadas
  currentTime = millis(); // Obtém o tempo atual em milissegundos
  previousTime = millis();
  double deltaTime = (currentTime - previousTime) / 1000.0; // Diferença de tempo em segundos
  currentSpeed = sqrt(pow((currentX - previousX) / deltaTime, 2) + pow((currentY - previousY) / deltaTime, 2));

  // Atualiza os valores anteriores para a próxima iteração
  previousX = currentX;
  previousY = currentY;
  previousTime = currentTime;
}
