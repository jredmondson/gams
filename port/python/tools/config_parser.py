from data_reader_interface import DataReaderFromFile
from data_reader_interface import DataReaderFromKB
from data_reader_interface import KnowledgeBaseCreator
import data_reader_interface
from plotting import Plotter
import yaml
import yamlloader
import sys
from collections import OrderedDict

file_path = 'port/python/tools/config.yaml'

def yaml_loader(filepath):
    """ Loads a yaml file """
    with open(filepath, 'r') as file_descriptor:
        # data = OrderedDict()
        data = yaml.load(file_descriptor, 
                        Loader= yamlloader.ordereddict.CLoader)
    return data

source_key = 'data_source'
capnp_schemas_location = 'schemas'

stk_file_source = 'stk_file'

kb_tranport_settings_key = 'transport'
kb_transport_type_key = 'transport_type'
kb_transport_hosts_key = 'hosts'
kb_name_key = 'kb_name'

queue_lenght_key = 'queue_length'
read_threads_key = 'read_threads'
thread_hertz_key = 'read_thread_hertz'

subkeys_key = 'knowledge_record_subkeys'
frames_key = 'frame_types'
any_type_key = 'any_types'


# TODO: also handle points per each graph

########################### Specify Source  ############################
# 1. Specify the path of the .yaml configuration file

def create_plotters_from_config(file_path):
    user_specs = yaml_loader(file_path)
    # user_specs = OrderedDict(yaml_loader(file_path))
    # user_specs = OrderedDict(user_specs)
    # 2. Grab schema file
    if user_specs[source_key][capnp_schemas_location]:
        schemas = user_specs[source_key][capnp_schemas_location]
    else:
        sys.exit('You have not specified schema files')

    # 3. Grab source from which to read data (.stk or live transport)
    # Checks to see what source is specified (For now cannot be both)
    if user_specs[source_key].has_key(stk_file_source):
        stk = user_specs[source_key][stk_file_source]
        reader = DataReaderFromFile(schemas, stk)
    elif user_specs[source_key][kb_tranport_settings_key][kb_transport_type_key]:
        transport_type = user_specs[source_key][kb_tranport_settings_key][kb_transport_type_key]
        hosts = user_specs[source_key][kb_tranport_settings_key][kb_transport_hosts_key]
        kb_name = user_specs[source_key][kb_tranport_settings_key][kb_name_key]
        queue_lenght = None
        thread_hertz = None
        read_threads = None

        transport_settings = user_specs[source_key][kb_tranport_settings_key]
        #sub_dict = dict(sub_dict)
        if transport_settings.has_key(queue_lenght_key):
            queue_lenght = transport_settings[queue_lenght_key]

        if transport_settings.has_key(read_threads_key):
            read_threads = transport_settings[read_threads_key]

        if transport_settings.has_key(thread_hertz_key):
            thread_hertz = transport_settings[thread_hertz_key]

        creator = KnowledgeBaseCreator(kb_name, transport_type, hosts, queue_lenght,
                            read_threads, thread_hertz)

        kb = creator.get_knowledge_base()

        reader = DataReaderFromKB(schemas, kb)
    else:
        print('You have not specified a data source to read from or it is incomplete!')
        sys.exit()


    # TODO: maybe merge types into one?
    ####################################### Plot Data ###############################################
    # 1. Create list of KRs for which the user would like to be plotted.
    # Separate lists for Any and Frame type data
    if user_specs[subkeys_key][frames_key]:
        frame_KRs = user_specs[subkeys_key][frames_key]   # list
    else:
        frame_KRs = {}
    if user_specs[subkeys_key][any_type_key]:
        any_KRs= user_specs[subkeys_key][any_type_key]     # list
    else:
        any_KRs = {}

    has_other_key = False
    plot_dict = {}

    for key in user_specs[subkeys_key]:
        if not (key == frames_key)  and not (key == any_type_key):
            has_other_key = True
            value = user_specs[subkeys_key][key]
            if not (value == None):
                sub_plot_list = []
                for subkey in value.keys():
                    if 'plot' in subkey:
                        sub_plot_list.append(value[subkey].items())
                plot_dict[key] = Plotter(reader, key, subkeys=sub_plot_list)
            else:
                plot_dict[key] = Plotter(reader, key)

    if len(any_KRs) == 0 and len(frame_KRs) == 0 and not (has_other_key):
        print('You have not specified any knowledge records!')
        sys.exit()

    # 2. Frame type data plotting
    for key, value in frame_KRs.items():
        sub_plot_list = []
        flagger = [0, 0, 0, 0]


        try: 
            if '3D' in value.keys() and value['3D']:
                flagger[0] += 1
            
            if 'points_per_plot' in value.keys():
                points = value['points_per_plot']
                flagger[1] += 1

            if 'reference_frame' in value.keys():
                reference_frames = value['reference_frame']
                flagger[2] += 1

            # just to find if 'plot_' substring is in a value string
            s = [s for s in value.keys() if 'plot_' in s]
            if s:
            # if 'plot_' in value.keys():
                flagger[3] += 1
                for subkey in value.keys():
                        if 'plot_' in subkey:
                            sub_plot_list.append(frame_KRs[key][subkey].items())

        except: 
            pass

        if flagger == [0, 0, 0, 0]:
            plot_dict[key] = Plotter(reader, data_reader_interface.frames_prefix + '.' + key)
            continue

        if flagger == [1, 0, 0, 0]:
            plot_dict[key] = Plotter(reader, data_reader_interface.frames_prefix + '.' + key, 
                                plot_to_3d=True)
            continue

        if flagger == [0, 1, 0, 0]:
            plot_dict[key] = Plotter(reader, data_reader_interface.frames_prefix + '.' + key, 
                                        points_per_plot=points)
            continue

        if flagger == [0, 0, 1, 0]:
            plot_dict[key] = Plotter(reader, data_reader_interface.frames_prefix + '.' + key, 
                                        frames_of_choice=reference_frames)
            continue

        if flagger == [0, 0, 0, 1]:
            plot_dict[key] = Plotter(reader, data_reader_interface.frames_prefix + '.' + key, 
                                        subkeys=sub_plot_list)
            continue

        if flagger == [1, 1, 0, 0]:
            plot_dict[key] = Plotter(reader, data_reader_interface.frames_prefix + '.' + key, 
                                        points_per_plot=points, plot_to_3d=True)
            continue

        if flagger == [0, 1, 1, 0]:
            plot_dict[key] = Plotter(reader, data_reader_interface.frames_prefix + '.' + key, 
                                        points_per_plot=points, frames_of_choice=reference_frames)
            continue

        if flagger == [0, 0, 1, 1]:
            plot_dict[key] = Plotter(reader, data_reader_interface.frames_prefix + '.' + key, 
                                        frames_of_choice=reference_frames, subkeys=sub_plot_list)
            continue

        if flagger == [1, 0, 1, 0]:
            plot_dict[key] = Plotter(reader, data_reader_interface.frames_prefix + '.' + key, 
                                        plot_to_3d=True, frames_of_choice=reference_frames)
            continue

        if flagger == [1, 0, 0, 1]:
            plot_dict[key] = Plotter(reader, data_reader_interface.frames_prefix + '.' + key, 
                                        plot_to_3d=True, subkeys=sub_plot_list)
            continue

        if flagger == [0, 1, 0, 1]:
            plot_dict[key] = Plotter(reader, data_reader_interface.frames_prefix + '.' + key, 
                                        points_per_plot=points, subkeys=sub_plot_list)
            continue

        if flagger == [1, 1, 1, 0]:
            plot_dict[key] = Plotter(reader, data_reader_interface.frames_prefix + '.' + key, 
                                        points_per_plot=points, plot_to_3d=True, 
                                        frames_of_choice=reference_frames)
            continue

        if flagger == [0, 1, 1, 1]:
            plot_dict[key] = Plotter(reader, data_reader_interface.frames_prefix + '.' + key, 
                                        points_per_plot=points, subkeys=sub_plot_list, 
                                        frames_of_choice=reference_frames)
            continue

        if flagger == [1, 0, 1, 1]:
            plot_dict[key] = Plotter(reader, data_reader_interface.frames_prefix + '.' + key, 
                                        plot_to_3d=True, subkeys=sub_plot_list, 
                                        frames_of_choice=reference_frames)
            continue
        
        if flagger == [1, 1, 0, 1]:
            plot_dict[key] = Plotter(reader, data_reader_interface.frames_prefix + '.' + key, 
                                        plot_to_3d=True, subkeys=sub_plot_list, 
                                        points_per_plot=points)
            continue

        if flagger == [1, 1, 1, 1]:
            plot_dict[key] = Plotter(reader, data_reader_interface.frames_prefix + '.' + key, 
                                        points_per_plot=points, plot_to_3d=True, 
                                        frames_of_choice=reference_frames, subkeys=sub_plot_list)
            continue


    # 3. Any type data plotting
    #TODO: Allow for handling of just specifying key and plotting subkeys automatically
    for key, value in any_KRs.items():
        sub_plot_list = []
        flagger = [0, 0, 0]

        try: 
            if '3D' in value.keys() and value['3D']:
                flagger[0] += 1
            
            if 'points_per_plot' in value.keys():
                points = value['points_per_plot']
                flagger[1] += 1

            # just to find if 'plot_' substring is in a value string
            s = [s for s in value.keys() if 'plot_' in s]
            if s:
                flagger[2] += 1
                for subkey, subvalue in value.items():
                        if 'plot_' in subkey:
                            try:
                                sub_plot_list.append(sorted(any_KRs[key][subkey].items()))

                            except:
                                sub_plot_list.append(subvalue)

        except: 
            pass

        if flagger == [0, 0, 1]:
            plot_dict[key] = Plotter(reader, key, subkeys=sub_plot_list)
            continue

        if flagger == [0, 1, 1]:
            plot_dict[key] = Plotter(reader, key, subkeys=sub_plot_list, points_per_plot=points)
            continue

        if flagger == [1, 0, 1]:
            plot_dict[key] = Plotter(reader, key, plot_to_3d=True, subkeys=sub_plot_list)
            continue

        if flagger == [1, 1, 1]:
            plot_dict[key] = Plotter(reader, key, plot_to_3d=True, subkeys=sub_plot_list,
                                    points_per_plot=points)
            continue

    return plot_dict