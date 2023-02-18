#ifndef RPCSensorAPI_H_
#define RPCSensorAPI_H_

#include <IRPC.h>

#include <cmath>

namespace RPCSensorAPI{

	//! Constants useful for default method ids (in case the exact identification of the exact call is not required)
	enum RemoteApiFunctions{
		UPDATE_ALTITUDE_AGL,//@Server
		UPDATE_EVENT,//@Server
		UPDATE_SENSOR_DATA,//@Horizon
		SET_OVERRIDE_VALUES,//@Horizon
		UPDATE_TRAFFIC_DATA,//@Horizon
		SET_GENERIC_VALUES,//@Horizon
		REMOTE_API_FUNCTION_COUNT
	};

	enum RemoteEvent{
		WARNING_LAMP_CLICKED,
		REMOTE_EVENT_COUNT
	};
	
	enum WarningLevel{
		GOOD,
		WARNING,
		BAD,
		WARNING_LEVEL_COUNT
	};
	
	//! a generic value with a warning level
	struct ValueWithLevel{
	
		std::string value;//! value as indicated
		int warningLevel;//! WarningLevel enum
		
		CREATE_BEGIN(ValueWithLevel)
			FILL_FIELD(value)
			FILL_FIELD(warningLevel)
		CREATE_END
		
		CREATE_NATIVE_BEGIN(ValueWithLevel)
			FILL_NATIVE_FIELD_IF_AVAILABLE(value, std::string())
			FILL_NATIVE_FIELD_IF_AVAILABLE(warningLevel, (int)(WarningLevel::BAD))
		CREATE_NATIVE_END
		
	};
	
	struct ValueWithFill{
		
		float fill;//! 0-1
		std::string value;//! value as indicated
		int warningLevel;//! WarningLevel enum
		
		CREATE_BEGIN(ValueWithFill)
			FILL_FIELD(fill)
			FILL_FIELD(value)
			FILL_FIELD(warningLevel)
		CREATE_END
		
		CREATE_NATIVE_BEGIN(ValueWithFill)
			FILL_NATIVE_FIELD_IF_AVAILABLE(fill, 0.f)
			FILL_NATIVE_FIELD_IF_AVAILABLE(value, std::string())
			FILL_NATIVE_FIELD_IF_AVAILABLE(warningLevel, (int)(WarningLevel::BAD))
		CREATE_NATIVE_END
		
	};
	
	//! all generic values
	struct GenericValues{
		
		std::unordered_map<std::string, ValueWithLevel> singleValues;
		std::unordered_map<std::string, std::vector<ValueWithFill> > multiValues;
		
		CREATE_BEGIN(GenericValues)
			FILL_FIELD(singleValues)
			FILL_FIELD(multiValues)
		CREATE_END
		
		CREATE_NATIVE_BEGIN(GenericValues)
			FILL_NATIVE_FIELD_IF_AVAILABLE(singleValues, {})
			FILL_NATIVE_FIELD_IF_AVAILABLE(multiValues, {})
		CREATE_NATIVE_END
		
	};
	
	//! specifies the range indication around the aircraft in the moving map
	struct RangeSpecification{
		
		enum Type{
			DISABLED,//! no range indication override
			FIXED_CIRCLE,//! circle around current position with given radius
			TYPE_COUNT
		};
		
		uint8_t type;//! see Type enum
		
		union{
			double radius;//! in meter
			// TODO: other values for other future specification types / set of parameters (see Type enum and functions below)
		};
		
		RangeSpecification(){
			type = DISABLED;
		}
		
		CREATE_BEGIN(RangeSpecification)
			FILL_FIELD(type)
			if(nativeValue.type==FIXED_CIRCLE){
				FILL_FIELD(radius)
			}
		CREATE_END
		
		CREATE_NATIVE_BEGIN(RangeSpecification)
			FILL_NATIVE_FIELD_IF_AVAILABLE(type, (uint8_t)DISABLED)
			if(nativeValue.type==FIXED_CIRCLE){
				FILL_NATIVE_FIELD_IF_AVAILABLE(radius, 0.0)
			}
		CREATE_NATIVE_END
		
	};
	
	struct OverrideValues{
		
		std::vector<float> batteryLevels;//! multiple levels will be indicated if more than one present
		std::vector<int32_t> batteryWarningLevels;//! warning levels for battery levels (standard warning colors will be used if unspecified)
		std::vector<int32_t> warningLevels;//! multiple warning colors will be indicated if more than one present (for warning lamp indication)
		RangeSpecification range;
		
		CREATE_BEGIN(OverrideValues)
			FILL_FIELD(batteryLevels)
			FILL_FIELD(batteryWarningLevels)
			FILL_FIELD(warningLevels)
			FILL_FIELD(range)
		CREATE_END
		
		CREATE_NATIVE_BEGIN(OverrideValues)
			FILL_NATIVE_FIELD_IF_AVAILABLE(batteryLevels, std::vector<float>{0.f})
			FILL_NATIVE_FIELD_IF_AVAILABLE(batteryWarningLevels, {})
			FILL_NATIVE_FIELD_IF_AVAILABLE(warningLevels, std::vector<int32_t>{WarningLevel::BAD})
			FILL_NATIVE_FIELD_IF_AVAILABLE(range, RangeSpecification())
		CREATE_NATIVE_END
		
	};
	
	//! not all values can be invalid, see comments below
	constexpr int32_t invalidValue = -100000;
	
	//! check if a double represents an invalid value
	inline bool isInvalid(double value){
		return fabs(value-invalidValue)<0.01;
	}

	struct RemoteSensorData{
		
		double longitude, latitude;//! in degrees, positive to north and east
		double altitude;//! true altitude in meter MSL e.g. from GNSS receiver (DO NOT USE pressure derived altitude)
		double groundspeed;//! groundspeed in m/s e.g. from GNSS receiver
		double groundtrack;//! groundtrack in degrees (true north), clockwise positive, e.g. from GNSS receiver
		uint32_t satCount;//! amount of GNSS satellites in use
		
		//! 3x3 attitude rotation matrix:
		//! | attitude[0] attitude[1] attitude[2] |
		//! | attitude[3] attitude[4] attitude[5] |
		//! | attitude[6] attitude[7] attitude[8] |
		//! Coordinate Systems (x,y,z), left handed:
		//! Aircraft: X to the right, Y to the top, Z to the front
		//! World: X: to the east, Y to the top / sky, Z to the north
		//! Heading / Yaw angle encoded in the matrix shall correspond to the groundtrack (see comment on indicatedHeading below)
		std::vector<double> attitude;
		
		//! Heading which is indicated in Horizon (true north), should be like the groundtrack (e.g. a groundtrack extrapolated using a gyroscope for smooth movements), DO NOT use magnetic heading here, also true heading is discouraged
		//! If the heading deviates significantly from the groundtrack, flying a tunnel in the sky (e.g. an approach procedure) will be much more difficult!
		double indicatedHeading;
		
		double trueHeading;//! in degrees (true north), or invalidValue
		
		double magneticHeading;//! in degrees (magnetic north), or invalidValue
		
		//! 3D acceleration in aircraft coordinate system
		std::vector<double> acceleration;
		
		double verticalSpeed;//! vertical speed in m/s
		
		double turnrate;//! turnrate in °/s (yaw rate)
		
		double airspeed;//! indicated / calibrated airspeed in m/s, invalidValue if not available
		
		double pressInside;//! in mbar, or invalidValue if not available
		
		double pressOutside;//! in mbar, or invalidValue if not available, required for true airspeed calculations
		
		double tempInside;//! in °C, or invalidValue if not available
		
		double tempOutside;//! in °C, or invalidValue if not available, required for true airspeed calculations
		
		double loc90, loc150;//! localizer 90Hz and 150Hz amplitude, invalidValue if no reception / not available
		double gs90, gs150;//! glideslope 90Hz and 150Hz amplitude, invalidValue if no reception / not available
		
		double aoa;//! angle of attack in degrees, or invalidValue
		double maxAoA;//! maximum angle of attack, or invalidValue
		
		int32_t warningLevel;//! from WarningLevel enum, will be displayed if no override set (see above)
		
		RemoteSensorData(){
			verticalSpeed = turnrate = indicatedHeading = longitude = latitude = altitude = groundspeed = groundtrack = 0.0;
			satCount = 0;
			attitude = std::vector<double>{1,0,0,0,1,0,0,0,1};
			acceleration = std::vector<double>{0.0,-9.81,0.0};
			airspeed = pressInside = pressOutside = tempInside = tempOutside = loc150 = loc90 = gs150 = gs90 = aoa = maxAoA = trueHeading = magneticHeading = invalidValue;
			warningLevel = WarningLevel::BAD;
		}
		
		CREATE_BEGIN(RemoteSensorData)
			FILL_FIELD(longitude)
			FILL_FIELD(latitude)
			FILL_FIELD(altitude)
			FILL_FIELD(groundspeed)
			FILL_FIELD(groundtrack)
			FILL_FIELD(satCount)
			FILL_FIELD(attitude)
			FILL_FIELD(indicatedHeading)
			FILL_FIELD(trueHeading)
			FILL_FIELD(magneticHeading)
			FILL_FIELD(acceleration)
			FILL_FIELD(verticalSpeed)
			FILL_FIELD(turnrate)
			FILL_FIELD(airspeed)
			FILL_FIELD(pressInside)
			FILL_FIELD(pressOutside)
			FILL_FIELD(tempInside)
			FILL_FIELD(tempOutside)
			FILL_FIELD(loc90)
			FILL_FIELD(loc150)
			FILL_FIELD(gs90)
			FILL_FIELD(gs150)
			FILL_FIELD(aoa)
			FILL_FIELD(maxAoA)
			FILL_FIELD(warningLevel)
		CREATE_END
		
		CREATE_NATIVE_BEGIN(RemoteSensorData)
			FILL_NATIVE_FIELD_IF_AVAILABLE(longitude, 0.0)
			FILL_NATIVE_FIELD_IF_AVAILABLE(latitude, 0.0)
			FILL_NATIVE_FIELD_IF_AVAILABLE(altitude, 0.0)
			FILL_NATIVE_FIELD_IF_AVAILABLE(groundspeed, 0.0)
			FILL_NATIVE_FIELD_IF_AVAILABLE(groundtrack, 0.0)
			FILL_NATIVE_FIELD_IF_AVAILABLE(satCount, 0)
			static const std::vector<double> defaultAttitude{1,0,0,0,1,0,0,0,1};
			FILL_NATIVE_FIELD_IF_AVAILABLE(attitude, defaultAttitude)
			FILL_NATIVE_FIELD_IF_AVAILABLE(indicatedHeading, 0.0)
			FILL_NATIVE_FIELD_IF_AVAILABLE(trueHeading, invalidValue)
			FILL_NATIVE_FIELD_IF_AVAILABLE(magneticHeading, invalidValue)
			static const std::vector<double> defaultAcc{0.0,-9.81,0.0};
			FILL_NATIVE_FIELD_IF_AVAILABLE(acceleration, defaultAcc)
			FILL_NATIVE_FIELD_IF_AVAILABLE(verticalSpeed, 0.0)
			FILL_NATIVE_FIELD_IF_AVAILABLE(turnrate, 0.0)
			FILL_NATIVE_FIELD_IF_AVAILABLE(airspeed, invalidValue)
			FILL_NATIVE_FIELD_IF_AVAILABLE(pressInside, invalidValue)
			FILL_NATIVE_FIELD_IF_AVAILABLE(pressOutside, invalidValue)
			FILL_NATIVE_FIELD_IF_AVAILABLE(tempInside, invalidValue)
			FILL_NATIVE_FIELD_IF_AVAILABLE(tempOutside, invalidValue)
			FILL_NATIVE_FIELD_IF_AVAILABLE(loc90, invalidValue)
			FILL_NATIVE_FIELD_IF_AVAILABLE(loc150, invalidValue)
			FILL_NATIVE_FIELD_IF_AVAILABLE(gs90, invalidValue)
			FILL_NATIVE_FIELD_IF_AVAILABLE(gs150, invalidValue)
			FILL_NATIVE_FIELD_IF_AVAILABLE(aoa, invalidValue)
			FILL_NATIVE_FIELD_IF_AVAILABLE(maxAoA, invalidValue)
			FILL_NATIVE_FIELD_IF_AVAILABLE(warningLevel, WarningLevel::BAD)
		CREATE_NATIVE_END
		
	};

	struct RemoteTrafficElement{
		
		uint32_t icaoAddress;//! first 3 Bytes form the address (0: n/A)
		double longitude, latitude;//! in degrees, positive to north and east, invalidValue if not available
		int32_t pressureAltitude;//! feet (invalidValue n/A) (pressure altitude, e.g. from ads-b)
		int32_t velocity;//! knots (invalidValue n/A)
		int32_t climbRate;//! feet/min (invalidValue n/A)
		int32_t squawk;//! 13 bits (invalidValue: n/A)
		double groundtrack;//! ° (invalidValue: n/A)
		std::string name;//! name / identifier (no longer than 9 characters)
		
		RemoteTrafficElement(){
			icaoAddress = 0;
			groundtrack = squawk = climbRate = velocity = pressureAltitude = longitude = latitude = invalidValue;
		}
		
		CREATE_BEGIN(RemoteTrafficElement)
			FILL_FIELD(icaoAddress)
			FILL_FIELD(longitude)
			FILL_FIELD(latitude)
			FILL_FIELD(pressureAltitude)
			FILL_FIELD(velocity)
			FILL_FIELD(climbRate)
			FILL_FIELD(squawk)
			FILL_FIELD(groundtrack)
			FILL_FIELD(name)
		CREATE_END
		
		CREATE_NATIVE_BEGIN(RemoteTrafficElement)
			FILL_NATIVE_FIELD_IF_AVAILABLE(icaoAddress, 0)
			FILL_NATIVE_FIELD_IF_AVAILABLE(longitude, invalidValue)
			FILL_NATIVE_FIELD_IF_AVAILABLE(latitude, invalidValue)
			FILL_NATIVE_FIELD_IF_AVAILABLE(pressureAltitude, invalidValue)
			FILL_NATIVE_FIELD_IF_AVAILABLE(velocity, invalidValue)
			FILL_NATIVE_FIELD_IF_AVAILABLE(climbRate, invalidValue)
			FILL_NATIVE_FIELD_IF_AVAILABLE(squawk, invalidValue)
			FILL_NATIVE_FIELD_IF_AVAILABLE(groundtrack, invalidValue)
			FILL_NATIVE_FIELD_IF_AVAILABLE(name, std::string())
		CREATE_NATIVE_END
	};

	// called by the server --------------------------------

	inline void updateTrafficData(IRPC* rpc, const std::vector<RemoteTrafficElement>& data, IRemoteProcedureCaller* caller = NULL, uint32_t id = UPDATE_TRAFFIC_DATA){
		rpc->callRemoteProcedure("updateTrafficData", std::vector<IRPCValue*>{createRPCValue(data)}, caller, id);
	}

	inline void updateSensorData(IRPC* rpc, const RemoteSensorData& data, IRemoteProcedureCaller* caller = NULL, uint32_t id = UPDATE_SENSOR_DATA){
		rpc->callRemoteProcedure("updateSensorData", std::vector<IRPCValue*>{createRPCValue(data)}, caller, id);
	}
	
	inline void setOverrideValues(IRPC* rpc, const OverrideValues& data, IRemoteProcedureCaller* caller = NULL, uint32_t id = SET_OVERRIDE_VALUES){
		rpc->callRemoteProcedure("setOverrideValues", std::vector<IRPCValue*>{createRPCValue(data)}, caller, id);
	}

	inline void setGenericValues(IRPC* rpc, const GenericValues& data, IRemoteProcedureCaller* caller = NULL, uint32_t id = SET_GENERIC_VALUES){
		rpc->callRemoteProcedure("setGenericValues", std::vector<IRPCValue*>{createRPCValue(data)}, caller, id);
	}

	// called by Horizon -----------------------------------
	
	//! altitude in meters above ground, invalidValue if unavailable
	inline void updateAltitudeAGL(IRPC* rpc, double altitude, IRemoteProcedureCaller* caller = NULL, uint32_t id = UPDATE_ALTITUDE_AGL){
		rpc->callRemoteProcedure("updateAltitudeAGL", std::vector<IRPCValue*>{createRPCValue(altitude)}, caller, id);
	}

	inline void updateEvent(IRPC* rpc, RemoteEvent event, IRemoteProcedureCaller* caller = NULL, uint32_t id = UPDATE_EVENT){
		rpc->callRemoteProcedure("updateEvent", std::vector<IRPCValue*>{createRPCValue((int)event)}, caller, id);
	}

};

#endif
