
from model.VehicleEventDispatcher import VehicleEventDispatcher

class V32VehicleStatusTranslator(object):

    def __init__(self, serial_data, vehicle_event_dispatcher):
        self._vehicle_event_dispatcher = vehicle_event_dispatcher
        
        splitted_data = serial_data.split(',')
        
        self._decode_motor_armed_property(splitted_data[0])
        self._decode_vehicle_attitude(splitted_data[1], splitted_data[2], splitted_data[3])
        self._decode_vehicle_altitude(splitted_data[4], splitted_data[5])
        self._decode_vehicle_altitude(splitted_data[4], splitted_data[5])
        self._decode_receiver_channels_values(splitted_data[6],
                                              splitted_data[7],
                                              splitted_data[8],
                                              splitted_data[9],
                                              splitted_data[10],
                                              splitted_data[11],
                                              splitted_data[12],
                                              splitted_data[13])
        self._decode_motor_command_throttle(splitted_data[14],
                                            splitted_data[15],
                                            splitted_data[16],
                                            splitted_data[17],
                                            splitted_data[18],
                                            splitted_data[19],
                                            splitted_data[20],
                                            splitted_data[21])
        self._decode_battery_voltage(splitted_data[22])
        self._decode_current_flight_mode(splitted_data[23])
        
    def _decode_motor_armed_property(self, motor_armed_property) :
        motor_armed = False
        if motor_armed_property == '1' :
            motor_armed = True
        self._vehicle_event_dispatcher.dispatch_event(VehicleEventDispatcher.MOTOR_ARMED_PROPERTY_EVENT, motor_armed)
        
    def _decode_vehicle_attitude(self, raw_roll, raw_pitch, raw_heading) :
        roll = float(raw_roll)
        self._vehicle_event_dispatcher.dispatch_event(VehicleEventDispatcher.VEHICLE_ROLL_PROPERTY_EVENT, roll)
        pitch = float(raw_pitch)
        self._vehicle_event_dispatcher.dispatch_event(VehicleEventDispatcher.VEHICLE_PITCH_PROPERTY_EVENT, pitch)
        heading = float(raw_heading)
        self._vehicle_event_dispatcher.dispatch_event(VehicleEventDispatcher.VEHICLE_HEADING_PROPERTY_EVENT, heading)
        
    def _decode_vehicle_altitude(self, raw_vehicle_altitude, altitude_hold_state) :
        vehicle_altitude = float(raw_vehicle_altitude)
        self._vehicle_event_dispatcher.dispatch_event(VehicleEventDispatcher.VEHICLE_ALTITUDE_PROPERTY_EVENT, vehicle_altitude)
        self._vehicle_event_dispatcher.dispatch_event(VehicleEventDispatcher.ALTITUDE_HOLD_STATE_PROPERTY_EVENT, altitude_hold_state)
        
    def _decode_receiver_channels_values(self, raw_roll,\
                                               raw_pitch,\
                                               raw_yaw,\
                                               raw_throttle,\
                                               raw_mode,\
                                               raw_aux1,\
                                               raw_aux2,\
                                               raw_aux3) :
        roll = int(raw_roll)
        self._vehicle_event_dispatcher.dispatch_event(VehicleEventDispatcher.RECEIVER_ROLL_PROPERTY_EVENT, roll)
        pitch = int(raw_pitch)
        self._vehicle_event_dispatcher.dispatch_event(VehicleEventDispatcher.RECEIVER_PITCH_PROPERTY_EVENT, pitch)
        yaw = int(raw_yaw)
        self._vehicle_event_dispatcher.dispatch_event(VehicleEventDispatcher.RECEIVER_YAW_PROPERTY_EVENT, yaw)
        throttle = int(raw_throttle)
        self._vehicle_event_dispatcher.dispatch_event(VehicleEventDispatcher.RECEIVER_THROTTLE_PROPERTY_EVENT, throttle)
        mode = int(raw_mode)
        self._vehicle_event_dispatcher.dispatch_event(VehicleEventDispatcher.RECEIVER_MODE_PROPERTY_EVENT, mode)
        aux1 = int(raw_aux1)
        self._vehicle_event_dispatcher.dispatch_event(VehicleEventDispatcher.RECEIVER_AUX1_PROPERTY_EVENT, aux1)
        aux2 = int(raw_aux2)
        self._vehicle_event_dispatcher.dispatch_event(VehicleEventDispatcher.RECEIVER_AUX2_PROPERTY_EVENT, aux2)
        aux3 = int(raw_aux3)
        self._vehicle_event_dispatcher.dispatch_event(VehicleEventDispatcher.RECEIVER_AUX3_PROPERTY_EVENT, aux3)

    def _decode_motor_command_throttle(self, raw_motor1_throttle,\
                                             raw_motor2_throttle,\
                                             raw_motor3_throttle,\
                                             raw_motor4_throttle,\
                                             raw_motor5_throttle,\
                                             raw_motor6_throttle,\
                                             raw_motor7_throttle,\
                                             raw_motor8_throttle) :
        motor1_throttle = int(raw_motor1_throttle)
        self._vehicle_event_dispatcher.dispatch_event(VehicleEventDispatcher.MOTOR1_THROTTLE_PROPERTY_EVENT, motor1_throttle)
        motor2_throttle = int(raw_motor2_throttle)
        self._vehicle_event_dispatcher.dispatch_event(VehicleEventDispatcher.MOTOR2_THROTTLE_PROPERTY_EVENT, motor2_throttle)
        motor3_throttle = int(raw_motor3_throttle)
        self._vehicle_event_dispatcher.dispatch_event(VehicleEventDispatcher.MOTOR3_THROTTLE_PROPERTY_EVENT, motor3_throttle)
        motor4_throttle = int(raw_motor4_throttle)
        self._vehicle_event_dispatcher.dispatch_event(VehicleEventDispatcher.MOTOR4_THROTTLE_PROPERTY_EVENT, motor4_throttle)
        motor5_throttle = int(raw_motor5_throttle)
        self._vehicle_event_dispatcher.dispatch_event(VehicleEventDispatcher.MOTOR5_THROTTLE_PROPERTY_EVENT, motor5_throttle)
        motor6_throttle = int(raw_motor6_throttle)
        self._vehicle_event_dispatcher.dispatch_event(VehicleEventDispatcher.MOTOR6_THROTTLE_PROPERTY_EVENT, motor6_throttle)
        motor7_throttle = int(raw_motor7_throttle)
        self._vehicle_event_dispatcher.dispatch_event(VehicleEventDispatcher.MOTOR7_THROTTLE_PROPERTY_EVENT, motor7_throttle)
        motor8_throttle = int(raw_motor8_throttle)
        self._vehicle_event_dispatcher.dispatch_event(VehicleEventDispatcher.MOTOR8_THROTTLE_PROPERTY_EVENT, motor8_throttle)
        
    def _decode_battery_voltage(self, raw_voltage) :
        voltage = float(raw_voltage)
        self._vehicle_event_dispatcher.dispatch_event(VehicleEventDispatcher.BATTERY_VOLTAGE_PROPERTY_EVENT, voltage)
    
    def _decode_current_flight_mode(self, raw_flight_mode) :
        flight_mode = 'Accro'
        if raw_flight_mode == '1' :
            flight_mode = "Stable"
        self._vehicle_event_dispatcher.dispatch_event(VehicleEventDispatcher.FLIGHT_MODE_PROPERTY_EVENT, flight_mode)
        