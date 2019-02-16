import madara
import madara.knowledge as engine
import madara.transport as transport
from madara.knowledge import Any
import gams.pose as gp
import os
import capnp

from datetime import datetime


#key_to_capnpfile = {key: value.to_any().tag() for key, value in reader if value.is_any_type()}

from time import sleep

toi_key_suffix = ".toi"
frames_prefix = '.gams.frames' # default prefix is gams frames

fes = gp.FrameEvalSettings()


# call this to override the default gams.frames key of frames
def set_frames_prefix(prefix):
  global frames_prefix
  frames_prefix = prefix

nano_size = 1000000000


class DataReaderInterface:
  capnp_registered_schemas = set()

  def __init__(self, capnp_folder):
    self.capnp_folder = capnp_folder

  # get all keys in a list
  def get_keys(self):
    pass

  # retrieve current value from current reader
  # returns (value, bool has_next) where value is the value for the given key
  # value can be KR type or dictionary
  # bool has_next describes if getting value for the same key next time can be meaningful
  # sometimes it might be obvious that no value with that key can be retrieved.
  # It is obvious for stk values

  def get_current_value(self, key, frames_of_choice=['geo', 'p1_base_stabilized']):
    pass 

  def capnp_crunch(self, key, value):
    schema = value.to_any().tag()
    if not (key in DataReaderInterface.capnp_registered_schemas):
      schema_file = capnp.load(self.capnp_folder + '/' + schema + '.capnp',
                               imports=[os.environ['CAPNP_ROOT'] + '/c++/src'])
      Any.register_class(schema, getattr(schema_file, schema))
      DataReaderInterface.capnp_registered_schemas.add(key)

    new_value = value.to_any().reader()
    return new_value

# this is an interface for simulating stk file as a knowledge base
class DataReaderFromFile(DataReaderInterface):

  # point out the stk file name and capnp schemas location
  def __init__(self, capnp_schemas_folder, file_name):
    DataReaderInterface.__init__(self, capnp_schemas_folder)


    self.settings = engine.CheckpointSettings()
    if not file_name:
      print "DataReaderFromFile: cannot init data reader, file name is not valid"

    self.settings.filename = (file_name)
    self.capnp_folder = capnp_schemas_folder
    self.knowledge_base = engine.KnowledgeBase()

    #retrieve all (key, value)s that can be plotted
    # since in checkpoints reader all records are returned sorted by their toi,
    # we don't need to sort our values later
    self.all_values = {}
    self.current_indexes = {}

    # this will contain all the KRs with gams frames ordered by toi
    self.gams_frames = []
    first = True
    first_toi = 0

    # load initially all the data so to not iterate over checkpoints each time.
    # This will occupy more memory, but will be faster
    # store all keys as well
    for key, value in engine.CheckpointReader(self.settings):
      if key and self.check_type_for_plotting(key, value):
        #gams_frames data retriveing simulation is a bit different

        if key.startswith(frames_prefix):
          self.gams_frames.append((key, value))
          if first:
            # this is the starting time
            first_toi = value.toi()
            first = False
          if not self.current_indexes.has_key(frames_prefix):
            self.current_indexes[frames_prefix] = -1
          continue

        if key.startswith(".gams"):
          #TODO: this requires more accurate handling
          # or making sure this is accurate
          # gams vars are being presented in a bit different way
          subkeys = key.split('.')

          if (len(subkeys) > 2):
            key = ''
            for i in range(0, len(subkeys) -2):
              if subkeys[i] != '':
                key += '.'
                key += subkeys[i]
        if first:
          # this is the starting time of simulation
          first_toi = value.toi()
          first = False
        if self.all_values.has_key(key):
          self.all_values[key].append(value)
        else:
          # init the first value
          self.all_values[key] = [value]

           # set the current index which is read
        if not self.current_indexes.has_key(key):
          self.current_indexes[key] = 0

    # current time in nanoseconds from utc(0)
    #TODO: check if there's a way to retrieve time in nanoseconds instead of multiplying by nano dimension
    self.init_time = int((datetime.utcnow() - datetime.utcfromtimestamp(0)).total_seconds() * nano_size)
    self.time_diff = self.init_time - first_toi
  # end of def __init__

  # retrieve all available keys
  def get_keys(self):
    return self.all_values.keys()
  # end of def get_keys


  # returns a pair, which consists of current value (simulates as live) for the given key
  # and a flag indicatingif the value is already the latest

  #frames_of_choice are gams frame, to know which frames_of_choice to retrieve
  def get_current_value(self, key, frames_of_choice=['geo', 'p1_base_stabilized']):
    # relative current time representing the simulation time passed from the first toi
    relative_current_time = int((datetime.utcnow() -
                                 datetime.utcfromtimestamp(0)).total_seconds()
                                * nano_size) - self.time_diff
    if key.startswith(frames_prefix):
      return self.get_gams_frame(frames_of_choice, relative_current_time)



    if not self.all_values.has_key(key):
      return None, False


    current_index = self.current_indexes[key]

    values_list = self.all_values[key]
    last_index = len(values_list) - 1

    # in case it has reached to the end, just return last value as current
    if current_index == last_index:
      value = values_list[last_index]
      if value.is_any_type():
        return self.get_capnp_value(key, values_list[last_index]), False
      else:
        return value, False

    # last argument is the relative current time
    index, value  = self.find_next_value(values_list, current_index, last_index, relative_current_time)

    self.current_indexes[key] = index
    if value.is_any_type():
      return self.get_capnp_value(key, value), True
    else:
      return value, True
  #end of def get_current_value


  # get the value that corresponds to the simluation current time and its index
  def find_next_value(self, values_list, first_index, last_index, relative_current_time):
    if values_list[first_index].toi() >= relative_current_time:
      return first_index, values_list[first_index]

    if values_list[last_index].toi() <= relative_current_time:
      return last_index, values_list[last_index]
  # end of def find_next_value



    # iterate over all the elements other than last one and first one
    for current_index in range(first_index, last_index - 1):

      if values_list[current_index + 1].toi() >= relative_current_time:
        return current_index, values_list[current_index]

    #again if not found, return the last one
    return last_index, values_list[last_index]
  # end of def find_next_value

  def get_gams_frame(self, frames_of_choice, relative_current_time):
    length = len(self.gams_frames)

    if length == 0:
      return None, False

    #last_index = -1
    index = self.current_indexes[frames_prefix] + 1
    while index < length:
      #   # most left value can require different handling
      #   # for now let's send None if no frame already with relative_current_time appears
      #   return None

      # if already at the next point
      if self.gams_frames[index][1].toi() > relative_current_time:
        break

      index += 1

    # set index back by one and break
    index -= 1

    # fill kb until last index with current simulation time
    for i in range(self.current_indexes[frames_prefix] + 1, index + 1):
      self.knowledge_base.set(self.gams_frames[i][0], self.gams_frames[i][1])

    self.current_indexes[frames_prefix] = index
    try:
      trans_frames = gp.ReferenceFrame.load_tree(self.knowledge_base,
                                                 madara.from_pystrings(frames_of_choice),
                                                 # this value represents -1 in python
                                                 18446744073709551615, fes)
      coord = trans_frames[1].origin().transform_to(trans_frames[0])
      if index == length - 1:
        return coord, False

      return coord, True

    except:
      # if exception, return None
      return None, True
  #end of def get_gams_frame

  # retrieve the dictionary for the capnp from appropriate schema
  # we have to handle capnp types
  def get_capnp_value(self, key, value):
    return self.capnp_crunch(key, value).to_dict()

  # end of def get_capnp_value



  # return true when type is something that can be plot
  # for now we consider that integer, double, and their arrays, as well as any times containig such values can be plot
  # if the key is toi then this is not a value to plot
  def check_type_for_plotting(self, key, value):
    if key.endswith(toi_key_suffix):
      return False

    # if gams frames or number types
    if key.startswith(frames_prefix) or value.is_integer_type() or value.is_double_type():
      return True

    elif value.is_any_type() and value.to_any().tag():
      # if capnp file then can have keys to be plotted
      return True

    return False

  # end of def check_type_for_plotting




# interface to retrieve data from kb
class DataReaderFromKB(DataReaderInterface):

  # constructor takes capnp schemas location and knowledge_base
  def __init__(self, capnp_schemas_folder, knowledge_base):
    DataReaderInterface.__init__(self, capnp_schemas_folder)

    if not knowledge_base:
      print "DataReaderFromKB: cannot init data reader, kb is not valid"
    self.knowledge_base = knowledge_base
    self.capnp_folder = capnp_schemas_folder
  # end of constructor

  def get_keys(self):
    if not self.knowledge_base:
      # if no valid kb return nothing
      return {}
    #TODO: fix this, to_map is not working
    #return self.knowledge_base.to_map()
    return None
  # end of def get_keys()

  # frames of choice can be
  # p1_base_link -> p1_base_stabilized -> p1_base_footprint -> p1_odom -> geo for gams

  # key is the key for the knowledge record we want to get
  def get_current_value(self, key, frames_of_choice=['geo', 'p1_base_stabilized']):
    # somehow no kb available
    if not self.knowledge_base:
      print "no valid knowledge base set"
      return None, False

    if key.startswith(frames_prefix):
      # gams frames are handled differently
      return self.get_frame(frames_of_choice)

    value = self.knowledge_base.get(key)

    if not value.exists():
      # none but may appear later
      return None, True





    if value.is_any_type():
      if value.to_any().tag():
        return self.get_capnp_value(key, value), True
      else:
        # might have value later
        return None, True
    return value, True

  # gams frames need a bit different handling, this returns gams frames coords
  def get_frame(self, frames_of_choice):
    try:
      trans_frames = gp.ReferenceFrame.load_tree(self.knowledge_base,
                                                 madara.from_pystrings(frames_of_choice),
                                                 # this value represents -1 in python
                                                 18446744073709551615, fes)
      coord = trans_frames[1].origin().transform_to(trans_frames[0])
      return coord, True

    except:
      # if exception, return None
      return None, True
  #end of def get_gams_frame

  # capnp values are handled seperately, they need to be extracted
  def get_capnp_value(self, key, value):
    return self.capnp_crunch(key, value).to_dict()
  # end of get_capnp_value
# end of data reader interface from KB


# Class to create KB
class KnowledgeBaseCreator():
  def __init__(self, kb_name, transport_type, hosts, queue_length=None,
               read_threads=None, read_thread_hertz=None):

    self.kb_name = kb_name
    # hosts must be not empty
    if len(hosts) == 0:
      print "hosts are not provided"
      return
    self.settings = transport.QoSTransportSettings()
    #TODO: add transports types here if new types are needed
    for host in hosts:
      self.settings.hosts.append(host)
    if transport_type == 'ZMQ':
      self.settings.type = transport.TransportTypes.ZMQ
    elif transport_type == 'UDP':
      self.settings.type = transport.TransportTypes.UDP
    elif transport_type == 'MULTICAST':
      self.settings.type = transport.TransportTypes.MULTICAST
    elif transport_type == 'BROADCAST':
      self.settings.type = transport.TransportTypes.BROADCAST

    if queue_length:
      self.settings.queue_length = queue_length
    if read_threads:
      self.settings.read_threads = read_threads
    if read_thread_hertz:
      self.settings.read_thread_hertz = read_thread_hertz

  # create a knowledge base with specified settings
  def get_knowledge_base(self):
    if self.settings:
      return engine.KnowledgeBase(self.kb_name, self.settings)
    return None