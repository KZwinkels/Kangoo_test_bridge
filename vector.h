//Vector VCCU structs

typedef struct {
	uint16_t	PlugPresent_Voltage;			//0x18FF1A80
	
	int			PlugLockPermission:2;			//0x18FF2182	
	int			PlugUnlockPermission:2;			//0x18FF2182
	int			ChargePermission:2;				//0x18FF2182
	int			IsolationMeasurementStatus:2;	//0x18FF2182
	int			ContactorStatus:2;				//0x18FF2182
	uint16_t	LinkVoltage;					//0x18FF2182
	uint16_t	ContactorVoltage;				//0x18FF2182
	
	uint8_t		StateMachineStatus;				//0x18FF5D80
	
	int			PlugPresent_Status:2;			//0x18FF1380
	int			Inlet_ConnectionStatus:2;		//0x18FF1380
	int			MotorStatus:3;					//0x18FF1380
	
	int			ChargeUnit_State:3;				//0x18FF1480
	int			ChargeUnit_Mode:2;				//0x18FF1480
	uint16_t	ControlPilot_Voltage;			//0x18FF1480
	
	int			ContactorRequest:2;				//0x18FF1780
	int			IsolationMeasurementRequest:2;	//0x18FF1780	
	
	int			EVMaximumVoltageLimitFlag:2;	//0x18FF3982
	int16_t		EVMaximumVoltageLimitValue;		//0x18FF3982
	int8_t		EVMaximumVoltageLimitMultiplier;//0x18FF3982
	int			EVMaximumVoltageLimitUnit:4;	//0x18FF3982
	uint8_t		EVMaximumVoltageLimitUnitFlag;	//0x18FF3982
	
	int			EVMaximumCurrentLimitFlag:2;	//0x18FF3782
	int16_t		EVMaximumCurrentLimitValue;		//0x18FF3782
	int8_t		EVMaximumCurrentLimitMultiplier;//0x18FF3782
	int			EVMaximumCurrentLimitUnit:4;	//0x18FF3782
	uint8_t		EVMaximumCurrentLimitUnitFlag;	//0x18FF3782
	
	int			EVMaximumPowerLimitFlag:2;		//0x18FF3882
	int16_t		EVMaximumPowerLimitValue;		//0x18FF3882
	int8_t		EVMaximumPowerLimitMultiplier;	//0x18FF3882
	int			EVMaximumPowerLimitUnit:4;		//0x18FF3882
	uint8_t		EVMaximumPowerLimitUnitFlag;	//0x18FF3882
	
	int			EVReady:2;						//0x18FF3082
	int			EVErrorCode:4;					//0x18FF3082
	uint8_t		EVRESSSOC;						//0x18FF3082
	int			EnergyTransferMode:4;			//0x18FF3082
	
	int16_t		EVTargetCurrentValue;			//0x18FF3482
	int8_t		EVTargetCurrentMultiplier;		//0x18FF3482
	int			EVTargetCurrentUnit:4;			//0x18FF3482
	uint8_t		EVTargetCurrentUnitFlag;		//0x18FF3482
	
	int16_t		EVTargetVoltageValue;			//0x18FF3382
	int8_t		EVTargetVoltageMultiplier;		//0x18FF3382
	int			EVTargetVoltageUnit:4;			//0x18FF3382
	uint8_t		EVTargetVoltageUnitFlag;		//0x18FF3382
	} vector_t;
	
enum Unit{H,M,S,A,Ah,V,VA,W,Ws,Wh};