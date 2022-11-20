#include "RPCSensorAPI.h"

#include <JSONRPC2Server.h>
#include <JSONRPC2Client.h>
#include <SimpleSockets.h>
#include <timing.h>
#include <Matrix.h>

#include <iostream>

#define RPC_PORT 48753

#define DEG_RAD (180.0/3.141592654)

using namespace RPCSensorAPI;

class ProtocolNegotiation : public IMetaProtocolHandler{

	public:
	
	bool tryNegotiate(ICommunicationEndpoint* socket) override{
		return true;//no negotiation required, always JSON-RPC
	}
	
	bool useCompression() const override{
		return true;//returns true to enable zlib compression of communication
	}

};

using Matrix3x3 = Matrix<3,3,double>;

Matrix3x3 createPitchMatrix(double phi){
	return Matrix3x3(	1, 0, 0,
		      			0, cos(phi), -sin(phi),
		      			0, sin(phi), cos(phi));
}

Matrix3x3 createYawMatrix(double phi){
	return Matrix3x3(	cos(phi), 0, sin(phi),
		     			0, 1, 0,
		     			-sin(phi), 0, cos(phi));
}

Matrix3x3 createRollMatrix(double phi){
	return Matrix3x3(	cos(phi), -sin(phi), 0,
		      			sin(phi), cos(phi), 0,
		     			 0, 0, 1);
}

class ProcedureHandler : public IRemoteProcedureCaller, IRemoteProcedureCallReceiver{
	
	RemoteSensorData sensorData;
	std::vector<RemoteTrafficElement> traffic;
	OverrideValues overrideValues;
	GenericValues genericValues;
	
	double lastSendTime;
	double lastTrafficSendTime;
	
	void fillGenericValues(){
		double t = getSecs();
		genericValues.singleValues["key"] = ValueWithLevel{std::to_string((int)t), ((int32_t)t)%3};
		genericValues.multiValues["multikey"] = std::vector<ValueWithFill>{{0.5+0.5*sin(2*3.14*0.25*t), std::to_string((int)t), ((int32_t)t)%3}, {0.5+0.5*cos(2*3.14*0.25*t), std::to_string((int)(10*t)), ((int32_t)t+1)%3}};
	}
	
	void fillOverrideValues(){
		double t = getSecs();
		overrideValues.batteryLevels = std::vector<float>{(float)(0.5+0.5*sin(2.0*3.14*0.25*t)), (float)(0.5+0.5*cos(2.0*3.14*0.25*t))};
		overrideValues.batteryWarningLevels = std::vector<int32_t>{((int32_t)t)%3, ((int32_t)t+1)%3};
		overrideValues.warningLevels = std::vector<int32_t>{((int32_t)t)%3, ((int32_t)t+1)%3};
	}
	
	void fillSensorData(){
		double t = getSecs();
		double p = sin(2.0*3.14*0.25*t);
		sensorData.longitude = 12.791+p*0.01;
		sensorData.latitude = 47.2916+p*0.01;
		sensorData.altitude = 1333+p*33;
		sensorData.groundspeed = 50+p*10;
		sensorData.groundtrack = fmod(t,36.0)*10.0;
		sensorData.satCount = 66;
		Matrix3x3 R = createYawMatrix(sensorData.groundtrack/DEG_RAD) * createPitchMatrix(p*30/DEG_RAD) * createRollMatrix(p*30/DEG_RAD);
		for(uint32_t i=0; i<9; i++){
			sensorData.attitude[i] = R.get(i/3, i%3);
		}
		sensorData.indicatedHeading = sensorData.groundtrack;
		sensorData.trueHeading = sensorData.groundtrack+3.0;
		sensorData.magneticHeading = sensorData.trueHeading+5.0;
		Vector3D<double> acc = createRollMatrix(-p*30/DEG_RAD) * Vector3D<double>(0.0, -9.81, 0.0);
		sensorData.acceleration = std::vector<double>{acc[0], acc[1], acc[2]};
		sensorData.verticalSpeed = p*5.0;
		sensorData.turnrate = p*10.0;
		sensorData.airspeed = sensorData.groundspeed;
		sensorData.pressInside = sensorData.pressOutside = 900.0 + p*3.0;
		sensorData.tempInside = invalidValue;
		sensorData.tempOutside = 25.0+p*5.0;
		sensorData.loc90 = 1.0+0.5*p;
		sensorData.loc150 = 1.0-0.5*p;
		sensorData.gs90 = sensorData.loc90;
		sensorData.gs150 = sensorData.loc150;
		sensorData.aoa = 5.0+5.0*p;
		sensorData.maxAoA = 10.0;
		int32_t w = ((int32_t)t)%3;
		sensorData.warningLevel = w;
	}
	
	void fillTrafficData(){
		double t = getSecs();
		double p = sin(2.0*3.14*0.5*t);
		RemoteTrafficElement& ele = traffic[0];
		ele.icaoAddress = 1234;
		ele.longitude = 12.791+p*0.01;
		ele.latitude = 47.2916+p*0.01;
		ele.pressureAltitude = 3000+p*100;
		ele.velocity = 100+p*20;
		ele.climbRate = p*500.0;
		ele.squawk = 896;//7000 octal
		ele.groundtrack = fmod(t,36.0)*10.0;
		ele.name = "D-TEST";
	}
	
	public:
	
	JSONRPC2Client* client;

	ProcedureHandler(JSONRPC2Client* client):traffic(1),client(client){
		lastTrafficSendTime = lastSendTime = getSecs();
		client->registerCallReceiver("updateAltitudeAGL", this);
		client->registerCallReceiver("updateEvent", this);
	}
	
	~ProcedureHandler(){
		delete client;
	}

	void OnProcedureResult(IRPCValue* results, uint32_t id){
		if(id==UPDATE_ALTITUDE_AGL){
			double altitude = createNativeValue<double>(results);
			std::cout << "Altitude AGL: " << altitude << std::endl;
		}else if(id==UPDATE_EVENT){
			int event = createNativeValue<int>(results);//see RemoteEvent enum
			std::cout << "Event received: " << event << std::endl;
		}
		delete results;
	}
	
	void OnProcedureError(int32_t errorCode, const std::string& errorMessage, IRPCValue* errorData, uint32_t id){
		std::cerr << "OnProcedureError: errorCode: " << errorCode << " errorMessage: " << errorMessage << " id: " << id << std::endl;
		if(errorData){std::cerr << convertRPCValueToJSONString(*errorData, true) << std::endl;}
		delete errorData;
	}

	IRPCValue* callProcedure(const std::string& procedure, const std::vector<IRPCValue*>& values){
		if(procedure=="updateAltitudeAGL"){
			double altitude = createNativeValue<double>(values[0]);
			std::cout << "Altitude AGL: " << altitude << std::endl;
		}else if(procedure=="updateEvent"){
			int event = createNativeValue<int>(values[0]);//see RemoteEvent enum
			std::cout << "Event received: " << event << std::endl;
		}
		return NULL;
	}
	
	void update(){
		client->update();
		double t = getSecs();
		if(t-lastSendTime>0.033){//30Hz for testing
			fillSensorData();
			updateSensorData(client, sensorData, this);
			fillOverrideValues();
			setOverrideValues(client, overrideValues, this);
			fillGenericValues();
			setGenericValues(client, genericValues, this);
			lastSendTime = t;
		}
		if(t-lastTrafficSendTime>0.5){
			fillTrafficData();
			updateTrafficData(client, traffic, this);
			lastTrafficSendTime = t;
		}
	}

};

int main(int argc, char *argv[]){
	
	ProtocolNegotiation pn;
	JSONRPC2Server server(RPC_PORT, 4000, &pn);
	
	std::list<ProcedureHandler> handlers;
	
	while(true){
		JSONRPC2Client* newClient = server.accept();
		if(newClient){
			handlers.emplace_back(newClient);
		}
		auto it = handlers.begin();
		while(it!=handlers.end()){
			ProcedureHandler& handler = *it;
			handler.update();
			if(handler.client->isConnected()){
				++it;
			}else{
				std::cout << "Client disconnected" << std::endl;
				it = handlers.erase(it);
			}
		}
		delay(10);
	}
	
	return 0;
}
