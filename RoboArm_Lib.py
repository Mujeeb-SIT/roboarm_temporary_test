'''
This class library contains the functionality to communiate with Robot Arm (i.e. send comamnds and listen to responses)

Author: Xing Yuan
Dependencies: 
1) python socket library
2) XOR_CheckSum library


Properties:
  list of public variable
Public Methods:
  list of public methods
Private Functions:
  list of private methods to be placed here
'''

#Import libraries
import socket
from XOR_CheckSum import xor_checksum_string

#'192.168.250.4'

#Todo: This class should be created as singleton...to avoid opening multiple connections to robot arm
class RoboArm:
  def __init__(self):
    self.port = None
    self.host = None
    self.connectionStatus = False
    self.socketHandler = None
    self.queue_tag_number = 1
   

  '''
    This function opens a socket connection to the robot arm.
    Commands are sent to the robot arm and responses are read from it over the socket link
    Socket link is using TCP. Robot arm does allow a UDP connection as well, but we are using TCP only

    Arguments: 
          host IP (provided by the calling function)
          tcp port: Currently robot software has fixed port-5890

    Return: [errorCode, description]
            error_code: 
            0:      Connection opened successfully
            Other:  Some error occured...Second item in the list is description
  '''

  def connect(self, host, port=5890):
    """
    Establishes
    :param host:
    :param port:
    :return:
    0 -> success
    1 -> failure
    2 -> Connection already exist
    """
    self.host = host
    self.port = port
    self.timeout_seconds = 6
    
    print("Inside RoboArm_Lib connect()...IP=",self.host)

    #if self.socketHandler != None: #if a socket connection already exists
    #self.disconnect()
    #self.socketHandler = None

    #open socket connection (again)
    if (self.connectionStatus == False):
      try:
        self.socketHandler = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socketHandler.connect((self.host, self.port))
        #self.socketHandler.settimeout(self.timeout_seconds)
        # self.socketHandler.setblocking(0)
        print("RoboArm_Lib...connection opened OK..", self.host, self.port)
        self.connectionStatus = True
        return [0, None]
      except socket.error as e:
        self.socketHandler = None
        return [-1, e]
    else:
      return [2,0] #Connection already exists

  '''
    This function closes a socket connection to the robot arm if already opened.
    
    Arguments: None
    Return: None
  '''
  def disconnect(self):
    if self.connectionStatus == False:
      print("disconnect()--- connection does not exists...returning")
      return

    try:
      self.socketHandler.shutdown()
      self.socketHandler.close()
    except Exception as e:
      pass
    finally:
      self.connectionStatus = False
      print("Disconnected")


  '''
    This function sends a command to move the robot arm 
  '''
  #def move(self, a0=90, a1=-90.0, a2=90, a3=0.0, a4=-90.0, a5=0.0, speed=50, accel=200):
  def move(self, a0=180, a1=-90.0, a2=90, a3=0.0, a4=-90.0, a5=0.0, speed=50, accel=200): #for new robot (different alignment)
    """
    TODO:
    - Safety check?
      if angle_round[0] <= 20 or angle_round[0] >= 150 or angle_wrist[0] <= -70 or angle_wrist[0] >= 70:
        print("Skipping:", angle_round[0], angle_wrist[0])
        time.sleep(0.01)
        continue

    :return:
    """

    self.queue_tag_number = _increase_queueTag(self.queue_tag_number)
    queue_tag_str = 'QueueTag(' + str(self.queue_tag_number) + ')\r\n'
    move_cmd_str = _create_PTP(a0, a1, a2, a3, a4, a5, speed, accel)

    payload = _package_cmd([move_cmd_str, queue_tag_str],self.queue_tag_number)

    if not payload:
      return [2, AttributeError("Error with command format")]

    try:
      self.socketHandler.send(payload.encode())

      return [0, None]
    except Exception as e:
      return [1, e]

  def response(self):
    # for a more robust code, check the checksum
    try:
      #response = self.socketHandler.recv(1024).decode()
      response = self.socketHandler.recv(128).decode() #Mjb
      temp = response.split('\r\n')
      temp2 = temp[0].split(',')
      output = temp2[0:-1]
      return output
    except Exception as e:
      return e


def _package_cmd(cmd_list: list, scriptID) -> str:
  if not all(isinstance(sub, str) for sub in cmd_list):
    return False

  data = str(scriptID)+','

  for cmd in cmd_list:
    data = data + cmd

  datasize = len(data)
  checksumstring = 'TMSCT,' + str(datasize) + ',' + data + ','
  checksum = hex(xor_checksum_string(checksumstring))[2:]

  if len(checksum) < 2:
    checksum = '0' + checksum

  payload = '$TMSCT,' + str(datasize) + ',' + data + ',*' + checksum + '\r\n'

  return payload


def _create_PTP(a0, a1, a2, a3, a4, a5, speed, accel):
  start_pos = [a0, a1, a2, a3, a4, a5, ]
  start_pos_str = str(start_pos)[1:-1]
  #cmd_str = "PTP" + '("JPP",' + start_pos_str + ',' + str(speed) + ',' + str(accel) + ',100,false) \r\n'
  cmd_str = "PTP" + '("JPP",' + start_pos_str + ',' + str(speed) + ',' + str(accel) + ',100,true) \r\n' #Mjb

  return cmd_str

def _increase_queueTag(number):
  if number == 15:
    return 1
  else:
    return number+1
