import matplotlib.pyplot as plt
import math
import data_reader_interface
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Button
from datetime import datetime
import numbers
import madara.knowledge

keys = []

class Plotter:
  #reader is the reader interface to be used to read data from stk file or kb

  # for subkeys we expect an array of mappings each value to another (this is for 2d maps)
  # if we want to set mapping to time, just setting the second index to -1
  # e.g. subkeys = []
  # subkeys.append([(0, 'x'), (1, 'y')])
  # subkeys.append([(2, 'z'), (-1, 'time')])
  # we expect second value always be a real index for subkeys
  # however it can be -1 so it can be plot against time
  # also 3d plotting can be done by passing 3 tuples in the array and set plot_to_3d to true

  # and pass this to the constructor
  # each value has a name to be shown as its axis name


  # frames_of_choice parameter is for gams frames only
  # it identifies the frames to load the tree, has a default value

  # points_per_plot indentifies the number of points to keep for plotting,
  # if 0, then no limit, it will keep all the retrieved polints

  def __init__(self, reader, key, subkeys = None, points_per_plot = 0, frames_of_choice=['geo', 'p1_base_stabilized'], plot_to_3d=False):
    if not reader or not key:
      print "Plotter is not constructed properly"
      return


    self.has_frames_key = False
    self.is_capnp_type = None
    self.plot_to_3d = plot_to_3d
    if key.startswith(data_reader_interface.frames_prefix):
      self.has_frames_key = True

    self.reader = reader
    self.name = key
    self.frames_of_choice=frames_of_choice

    self.plot_name = key
    # adjust key if reusing, to avoid plot collisions
    if key in keys:
      #this might be not a good solution, but at least we will skip collisions
      self.plot_name += ' ' * len(keys)
    keys.append(self.plot_name)

    self.subkeys = subkeys
    self.plot_3d = False
    self.number_of_points_per_plot = points_per_plot

    self.values = {}
    if subkeys:
      self.adjust_rows_and_columns(len(subkeys))

    else: # in this case plot the given key in one subplot
      self.number_of_rows = 1
      self.number_of_columns = 1

  def __del__(self):
    keys.remove(self.plot_name)



  # this function creates or updates the plot for the provided config of plotter
  def visualize(self):
    fig = plt.figure(self.plot_name)
    plt.clf()
    value, has_next = self.reader.get_current_value(self.name, self.frames_of_choice)
    if (value == None) or not has_next:
      return

    if self.is_capnp_type == None:
      self.is_capnp_type = isinstance(value, dict)


    if self.has_frames_key:
      new_values = [float(i) for i in value.to_string().split(',')]
      value = new_values

    if self.subkeys:
      for i in range(0, len(self.subkeys)):
        self.visualize_subkey(value, i)
    else:
      self.visualize_key(value, self.name)

    plt.pause(0.000000001)


  # visualizes a certain item from the given subkeys list
  # not to be called from outside
  def visualize_subkey(self, value, index):
    subkey = self.subkeys[index]
    # plot all of values inside subkey

    if self.is_capnp_type:
      # capnp values have a bit different presentation so they are handled seperately
      self.visualize_capnp_value(value, subkey, index)
      return



    #  parse as 3d if possible
    plot_to_3d = self.plot_to_3d and (len(subkey) == 3)
    if plot_to_3d:
      plt.subplot(self.number_of_rows, self.number_of_columns, index + 1, xlabel=subkey[0][1],
                  ylabel=subkey[1][1], zlabel=subkey[2][1], projection='3d')
      if not (self.values.has_key(index)):
        self.values[index] = ([], [], [])
    else:
      plt.subplot(self.number_of_rows, self.number_of_columns, index + 1, xlabel=subkey[0][1],
                  ylabel=subkey[1][1])
      if not (self.values.has_key(index)):
        self.values[index] = ([], [])

    current_time = int(
      (datetime.utcnow() - datetime.utcfromtimestamp(0)).total_seconds() * data_reader_interface.nano_size)
    for i in range(0, len(subkey)):
      if subkey[i][0] == -1:
        current_time = int(
          (datetime.utcnow() - datetime.utcfromtimestamp(0)).total_seconds() * data_reader_interface.nano_size)
        self.values[index][i].append(current_time)
      elif self.has_frames_key:
        self.values[index][i].append(value[subkey[i][0]])
      else:
        self.values[index][i].append(value.retrieve_index(subkey[i][0]).to_double())

    if self.number_of_points_per_plot > 0 and len(self.values[index][0]) > self.number_of_points_per_plot:
      self.values[index][0].pop(0)
      self.values[index][1].pop(0)
      if plot_to_3d:
        self.values[index][2].pop(0)

    if plot_to_3d:
      plt.plot(self.values[index][0], self.values[index][1], self.values[index][2])
    else:
      plt.plot(self.values[index][0], self.values[index][1])



  # visualizing capnp is a bit different, parsing keys and setting into the
  def visualize_capnp_value(self, value, subkey, index):
    # the second one is always a valid value
    if not isinstance(subkey, list):
      subvalue = self.get_value_for_key(subkey, value)
      # in this case we expect only one subkey, so we visualize that
      self.visualize_key(subvalue, subkey[0])
      return

    # parse as 3d if possible
    plot_to_3d = self.plot_to_3d and (len(subkey) == 3)
    if plot_to_3d:
      plt.subplot(self.number_of_rows, self.number_of_columns, index + 1, xlabel=subkey[0][1],
                  ylabel=subkey[1][1], zlabel=subkey[2][1], projection='3d')
      if not (self.values.has_key(index)):
        self.values[index] = ([], [], [])
    else:
      plt.subplot(self.number_of_rows, self.number_of_columns, index + 1, xlabel=subkey[0][1],
                  ylabel=subkey[1][1])
      if not (self.values.has_key(index)):
        self.values[index] = ([], [])

    current_time = int(
      (datetime.utcnow() - datetime.utcfromtimestamp(0)).total_seconds() * data_reader_interface.nano_size)
    for i in range(0, len(subkey)):
      item = subkey[i]

      if subkey[i][0] == -1:
        current_time = int(
          (datetime.utcnow() - datetime.utcfromtimestamp(0)).total_seconds() * data_reader_interface.nano_size)
        self.values[index][i].append(current_time)
      else:
        subvalue = self.get_value_for_key(item[0], value)
        self.values[index][i].append(subvalue)

    # if list is longer than the nuber of points we want to have on the plot, just pop the oldest ones
    if self.number_of_points_per_plot > 0 and len(self.values[index][0]) > self.number_of_points_per_plot:
      self.values[index][0].pop(0)
      self.values[index][1].pop(0)
      if plot_to_3d:
        # in 3d we have 3rd index as well to be popped
        self.values[index][2].pop(0)

    if plot_to_3d:
      plt.plot(self.values[index][0], self.values[index][1], self.values[index][2])
    else:
      plt.plot(self.values[index][0], self.values[index][1])




  # visualizes a single key (or a subkey) with no list of subkeys provided
  # this is a general way to plot values being called from most of places
  # handles several cases
  # 1. if the value is a dictionay,
  # 2. if the value is a list
  # 3. if the value is KR array
  # 4. if the value is number or a KR of a number type
  def visualize_key(self, value, key):
    if not (self.values.has_key(key)):
      self.values[key] = {}

    current_time = int(
      (datetime.utcnow() - datetime.utcfromtimestamp(0)).total_seconds() * data_reader_interface.nano_size)
    # if dictionary (can happen for capnp types)



    # the case when value is a dictionary
    if isinstance(value, dict):
      size = len(value.keys())
      plot_key_to_3d = self.plot_to_3d and (size >= 3)

      if not plot_key_to_3d:
        if (len(self.values[key]) == 0):
          self.adjust_rows_and_columns(len(value.keys()))
        else:
          self.adjust_rows_and_columns(len(self.values[key]))
      else:
        # keep a list of names to show as axes on 3d
        labels = []
      index = 1


      for curr_subkey in value.keys():
        curr_value = value[curr_subkey]

        # plot only number types
        if isinstance(curr_value, numbers.Number):
          if not (self.values[key].has_key(curr_subkey)):
            self.values[key][curr_subkey] = ([], [])

          # keeping always value and time lets switching between 3d and non 3d plottings
          self.values[key][curr_subkey][0].append(current_time)
          self.values[key][curr_subkey][1].append(curr_value)

          if self.number_of_points_per_plot > 0 and len( self.values[key][curr_subkey][0]) > self.number_of_points_per_plot:
            self.values[key][curr_subkey][0].pop(0)
            self.values[key][curr_subkey][1].pop(0)

          if not plot_key_to_3d:
            plt.subplot(self.number_of_rows, self.number_of_columns, index, xlabel='time', ylabel=curr_subkey)
            plt.plot(self.values[key][curr_subkey][0], self.values[key][curr_subkey][1])
          else:
            labels.append(curr_subkey)
            if index == 3:
              # 3 values already gathered
              # for 3d plotting we need 3d only at least for now
              break

          index += 1
      if plot_key_to_3d:
        plt.subplot(self.number_of_rows, self.number_of_columns, 1, xlabel=labels[0], ylabel=labels[1], zlabel=labels[2], projection='3d')
        plt.plot(self.values[key][labels[0]][1], self.values[key][labels[1]][1], self.values[key][labels[2]][1])
      return




    # the case when the value is an array
    elif isinstance(value, list):
      lenght = len(value)
      plot_key_to_3d = self.plot_to_3d and (lenght >= 3)
      if not plot_key_to_3d:
        self.adjust_rows_and_columns(lenght)
      else:
        labels = []
      for i in range(0, lenght):
        if not (self.values[key].has_key(i)):
          self.values[key][i] = ([], [])
        y_label = (key + '.' + str(i))
        self.values[key][i][1].append(value[i])
        self.values[key][i][0].append(current_time)

        # remove additional values
        if self.number_of_points_per_plot > 0 and len( self.values[key][i][0]) > self.number_of_points_per_plot:
          self.values[key][i][0].pop(0)
          self.values[key][i][1].pop(0)

        if not plot_key_to_3d:
          plt.subplot(self.number_of_rows, self.number_of_columns, i + 1, xlabel='time', ylabel=y_label)
          plt.plot(self.values[key][i][0], self.values[key][i][1])
        else:
          labels.append(y_label)
          if i == 2:
            # 2 means 3 values already gathered
            # for 3d plotting we need 3d only at least for now
            break
      if plot_key_to_3d:
        plt.subplot(self.number_of_rows, self.number_of_columns, 1, xlabel=labels[0], ylabel=labels[1], zlabel=labels[2], projection='3d')
        plt.plot(self.values[key][0][1], self.values[key][1][1], self.values[key][2][1])

      return



    # if the value is a single number, not a list, not a dictionary and not a KR
    #elif isinstance(value, numbers.Number):
    elif isinstance(value, madara.knowledge.KnowledgeRecord) and value.is_array_type():
    # value is KR, check if array type
    # similar to above one, but rather need to read value in a bit different way
    #elif value.is_array_type():
      # try to plot all the values of the array against time
      size = value.size()
      plot_key_to_3d = self.plot_to_3d and (size >= 3)
      if not plot_key_to_3d:
        self.adjust_rows_and_columns(size)
      else:
        labels = []
      for i in range(0, size):
        if not (self.values[key].has_key(i)):
          self.values[key][i] = ([], [])
        y_label = (key + '.' + str(i))
        self.values[key][i][1].append(value.retrieve_index(i).to_double())
        self.values[key][i][0].append(value.toi())

        # remove additional values
        if self.number_of_points_per_plot > 0 and len( self.values[key][i][0]) > self.number_of_points_per_plot:
          self.values[key][i][0].pop(0)
          self.values[key][i][1].pop(0)

        if not plot_key_to_3d:
          plt.subplot(self.number_of_rows, self.number_of_columns, i + 1, xlabel='time', ylabel=y_label)
          plt.plot(self.values[key][i][0], self.values[key][i][1])
        else:
          labels.append(y_label)
          if i == 2:
            # 2 means 3 values already gathered
            # for 3d plotting we need 3d only at least for now
            break
      if plot_key_to_3d:
        plt.subplot(self.number_of_rows, self.number_of_columns, 1, xlabel=labels[0], ylabel=labels[1], zlabel=labels[2], projection='3d')
        plt.plot(self.values[key][0][1], self.values[key][1][1], self.values[key][2][1])

      return


    # if KR is a number
    elif isinstance(value, madara.knowledge.KnowledgeRecord):
      # it should be a number type
      # first keep toi
      #TODO: confirm we want to use toi instead of current_time in general,
      # because when plotting it might be useful to plot against real/simulated time instead
      current_time = value.toi()
      value = value.to_double()
    # handle the general case when a number or a KR as a number is coming
    # it should be a number type
    plt.subplot(self.number_of_rows, self.number_of_columns, 1, ylabel=key, xlabel='time')
    if not (self.values[key].has_key(key)):
      # looks redundant having same key in key, but this lets to keep the structure a bit more generic
      self.values[key][key] = ([], [])

    self.values[key][key][1].append(value)
    self.values[key][key][0].append(current_time)

    if self.number_of_points_per_plot > 0 and len(self.values[key][key][0]) > self.number_of_points_per_plot:
      self.values[key][key][0].pop(0)
      self.values[key][key][1].pop(0)

    plt.plot(self.values[key][key][0], self.values[key][key][1])
  #
  #
  # end of visualize_key




  # adjust number of rows and columns according to the subplots lenght
  # to not get out of bounds of subplots list in case
  def adjust_rows_and_columns(self, lenght):

    rows = math.sqrt(lenght)
    self.number_of_rows = int(rows)
    self.number_of_columns = self.number_of_rows

    # try to make subplots plots in a square form grid
    # so all the subplots are visible in a better way
    # so adjust it twise if needed
    # first adjust
    if self.number_of_rows * self.number_of_columns < lenght:
      self.number_of_columns += 1
    # second adjust
    if self.number_of_rows * self.number_of_columns < lenght:
      self.number_of_rows += 1


  # for a capnp type objects parse the key and retrieve the value
  def get_value_for_key(self, key, value):
    # for dictionaries subkeys can be represented as
    # 'key1.[index1].key2.key3' it can has keys inside keys and indexes bounded by `[]`
    subkeys = key.split('.')
    if len(subkeys) == 0:
      # shall never happen
      return None

    # get the first value from map as subvalue_2
    if subkeys[0].startswith('[') and subkeys[0].endswith(']'):
      subvalue = value[int(subkeys[0][1:-1])]
    else:
      subvalue = value[subkeys[0]]

    for i in range(1, len(subkeys)):
      if subkeys[i].startswith('[') and subkeys[i].endswith(']'):
        subvalue = subvalue[int(subkeys[i][1:-1])]
      else:
        subvalue = subvalue[subkeys[i]]

    return subvalue