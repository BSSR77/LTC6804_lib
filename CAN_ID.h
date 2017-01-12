/*
 * CAN_ID.h
 *
 *  Created on: Dec 27, 2016
 *      Author: Frank Gu
 */

#ifndef CAN_ID_H_
#define CAN_ID_H_

#define CAN_HB_DLC 	 	 4							// Heartbeat frame data length
#define CAN_FW_DLC		 4							// Firmware version data length
#define CMD_DLC			 1							// Command data length
#define SW_Sentinel		 0xFFFFFFFF					// Sentinel indicating bad status word
#define HB_Interval		 1000						// CAN Heart-beat interval (ticks)
#define FW_Interval		 50							// Firmware broadcast interval (ticks)
/* CAN NodeID and associated ID organizations
 * P2P ID: nodeID + p2pOffest
 * Status Word ID: nodeID + swOffset
 * Firmware Version ID: nodeID + fwOffset
 */
// Offsets BEGIN
#define p2pOffset	 0x040
#define swOffset 	 0x050
#define fwOffset	 0x180
// Offsets END

// NodeIDs BEGIN
#define cc_nodeID		 1							// Command center nodeID
#define mc_nodeID		 2							// Motor controller nodeID
#define bps_nodeID		 3							// Battery protection system nodeID
#define ads_nodeID		 4						  // Array diagnostic system nodeID
#define radio_nodeID 	 6							// Radio module nodeID

// NodeIDs END

// P2P IDs BEGIN
#define cc_P2P			cc_nodeID + p2pOffset
#define mc_P2P			mc_nodeID + p2pOffset
#define bps_P2P			bps_nodeID + p2pOffset
#define ads_P2P			ads_nodeID	+ p2pOffset
#define radio_P2P		radio_nodeID + p2pOffset
// P2P IDs END

// Status Word IDs BEGIN
#define cc_SW				cc_nodeID + swOffset
#define mc_SW				mc_nodeID + swOffset
#define bps_SW			bps_nodeID + swOffset
#define ads_SW			ads_nodeID	+ swOffset
#define radio_SW		radio_nodeID + swOffset
// Status Word IDs END

// Firmware Revision IDs BEGIN
#define cc_FW				cc_nodeID	+ fwOffset
#define mc_FW				mc_nodeID	+ fwOffset
#define bps_FW			bps_nodeID + fwOffset
#define ads_FW			ads_nodeID + fwOffest
#define radio_FW		radio_nodeID + fwOffset		// Radio module firmware

// Firmware Revision IDs END

// Enumerations
// Connection state
typedef enum {
	DISCONNECTED,
	CONNECTING,
	CONNECTED,
	UNRELIABLE,
	CONN_ERROR
} connState;

// Node state
typedef enum {
	INIT,
	ACTIVE,
	SHUTDOWN,
	HARD_ERROR
} nodeState;

typedef enum {
	NODE_HRESET,
	NODE_RESET,
	NODE_SHUTDOWN,
	NODE_START,
	CC_ACK,
	CC_NACK
} nodeCommands;

// Error Messages
#define SysEMSD			0x10		// System emergency shutdown (Hard)
#define UsrEMSD			0x11		// User emergency shutdown (Hard)
#define bpsTrip			0x12		// BPS Trip condition (soft)
#define mcFault			0x13		// Motor controller fault

// Car Control Variables
#define swPos			0x21		// DCI Switch positions
#define swPos_DLC		0x2
#define brakePos		0x22		// DCI Brake position
#define brakePos_DLC	0x4
#define accelPos		0x23		// DCI Accelerator position
#define accelPos_DLC	0x4
#define regenPos		0x24		// DCI Regeneration position
#define regenPos_DLC	0x4

// Radio Received Commands
#define remoteSD		0x30		// Radio controlled car shutdown (soft)
#define setSpeed		0x31		// Set target cruise speed

// Motor Controller Diagnostics
#define mcDiag0			0x190		// 3 Diagnostic frames starting at 0x190
#define mcDiag1			0x191
#define mcDiag2			0x192

// Energy metering variables
#define battQCount		0x200		// Battery net Coulomb count
#define battPwr			0x201		// Battery power
#define motorPwr		0x202		// Motor power
#define lpBusPwr		0x203		// Low power bus power

#define pptAPwr			0x20A		// PPT A Power
#define pptBPwr			0x20B		// PPT B Power
#define pptCPwr			0x20C		// PPT C Power

// Module voltage offset
#define voltOffset		0x350		// Note the index of first module voltage at 0x351

// Module temperature array offset
#define tempOffset 		0x500		// Note the index of first temperature is at 0x501

// Mitsuba IDs (using Rear Right)
#define mitsubaREQ			Log_Req_RR1
#define mitsubaFr0			Log_Res_Frm0_RR1
#define	mitsubaFr1			Log_Res_Frm1_RR1
#define mitsubaFr2			Log_Res_Frm2_RR1

// Mitsuba controller diagnostic frame EXT IDs
#define Log_Req_RL1			0x08F89540
#define Log_Req_RR1			0x08F91540
#define Log_Req_FL1			0x08F99540
#define Log_Req_FR1			0x08FA1540

#define Log_Res_Frm0_RL1	0x08850225
#define Log_Res_Frm0_RR1	0x08850245
#define Log_Res_Frm0_FL1	0x08850265
#define Log_Res_Frm0_FR1	0x08850285

#define Log_Res_Frm1_RL1	0x08950225
#define Log_Res_Frm1_RR1	0x08950245
#define Log_Res_Frm1_FL1	0x08950265
#define Log_Res_Frm1_FR1	0x08950285

#define Log_Res_Frm2_RL1	0x08A50225
#define Log_Res_Frm2_RR1	0x08A50245
#define Log_Res_Frm2_FL1	0x08A50265
#define Log_Res_Frm2_FR1	0x08A50285

#define Req_DLC				0x1
#define Req_Frm0			0b001
#define Req_Frm1			0b010
#define Req_Frm2			0b100

#endif /* CAN_ID_H_ */
