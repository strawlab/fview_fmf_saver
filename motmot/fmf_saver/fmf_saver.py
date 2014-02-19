import time
import traits.api as traits
import motmot.fview.traited_plugin as traited_plugin
import numpy as np
from traitsui.api import View, Item, Group
import traits.api as traits
import threading
import motmot.FlyMovieFormat.FlyMovieFormat as FlyMovieFormat

# For a tutorial on Chaco and Traits, see
# http://code.enthought.com/projects/chaco/docs/html/user_manual/tutorial_2.html

def worker_func(save_info):
    filename = save_info['fname_prefix'] + time.strftime( '%Y%m%d_%H%M%S.fmf' )
    print 'saving to',filename
    fly_movie = FlyMovieFormat.FlyMovieSaver(filename,
                                             version=3,
                                             format='MONO8',
                                             bits_per_pixel=8,
                                             )
    for i in range(save_info['saved_count']):
        frame = save_info['buffer'][i,:,:]
        stamp = save_info['stamps'][i]
        fly_movie.add_frame(frame,stamp)
    fly_movie.close()
    print 'saved',filename

class FmfSaver(traited_plugin.HasTraits_FViewPlugin):
    plugin_name = 'ROS FMF saver'

    have_ros     = traits.Bool(False)    
    frames_buffered = traits.Int(0)    
    buffer_size     = traits.Int(0)    

    # set upon camera connection
    pixel_format = traits.String(None,transient=True)
    width = traits.Int(-1)
    height = traits.Int(-1)

    traits_view = View(
        Group(
            Item(name='have_ros',style='readonly'),
            Item(name='frames_buffered',style='readonly'),
            Item(name='buffer_size',style='readonly'),
            ),
        )

    def __init__(self,wx_parent,fview_options):
        super(FmfSaver,self).__init__(wx_parent)
        self._wx_parent = wx_parent
        self.have_ros = False

        self._save_lock = threading.Lock()
        self._save_info = None

        if fview_options.get('have_ros'):
            import roslib.packages
            try:
                roslib.load_manifest('ros_fview_fmf_saver')
                self.have_ros = True
            except roslib.packages.InvalidROSPkgException:
                pass

        if self.have_ros:
            import rospy
            import ros_fview_fmf_saver
            import ros_fview_fmf_saver.srv
            rospy.Service('~start_saving_fmf', ros_fview_fmf_saver.srv.StartSavingFMF, self.start_saving)

    def start_saving(self, req):
        # FIXME: why do I have to do this here? ----
        import ros_fview_fmf_saver.srv
        import std_msgs.msg
        # ------------------------------------------

        resp = ros_fview_fmf_saver.srv.StartSavingFMFResponse()

        if self.width <0 or self.height<0:
            resp.started_ok.data = False
            resp.error_message.data = 'camera not started'
            return resp

        alloc_size = req.allocate_buffer_size.data,self.height,self.width
        try:
            new_buf = np.empty( alloc_size, dtype=np.uint8 )
            stamps = np.empty( (req.allocate_buffer_size.data,), dtype=np.float )
        except MemoryError as err:
            resp.started_ok.data = False
            resp.error_message.data = str(err)
            return resp

        now = time.time()
        with self._save_lock:
            self._save_info = {'fname_prefix':req.fname_prefix.data,
                               'buffer':new_buf,
                               'stamps':stamps,
                               'nth_frame':req.nth_frame.data,
                               'save_framenumbers_as_timestamps':req.save_framenumbers_as_timestamps.data,
                               'saved_count':0,
                               'frame_count':0,
                               'start_time' : now,
                               }
            q = self._save_info
            bigbuf = q['buffer']

        resp.started_ok.data = True
        resp.error_message.data = ''
        return resp

    def initiate_stop_saving(self):
        with self._save_lock:
            q = self._save_info # shorthand
            self._save_info = None # no more saving in main thread
        save_worker = threading.Thread(target=worker_func, args=(q,))
        save_worker.start()

    def camera_starting_notification(self,cam_id,
                                     pixel_format=None,
                                     max_width=None,
                                     max_height=None):
        self.pixel_format = pixel_format
        assert self.pixel_format=='MONO8', 'Only MONO8 pixels currently supported'
        self.width = max_width
        self.height = max_height

    def process_frame(self,cam_id,buf,buf_offset,timestamp,framenumber):
        draw_points = []
        draw_linesegs = []

        initiate_stop = False

        with self._save_lock:
            q = self._save_info # shorthand
            if q is not None:
                newbuf = np.array(buf) # ensure this is a standard numpy array

                frame_count = q['frame_count']
                nth_frame = q['nth_frame']
                bigbuf = q['buffer']
                saved_count = q['saved_count']
                if q['save_framenumbers_as_timestamps']:
                    stamp_value = framenumber
                else:
                    stamp_value = timestamp

                if (frame_count%nth_frame==0):
                    bigbuf[saved_count] = newbuf # copy data
                    q['stamps'][saved_count] = stamp_value
                    q['saved_count'] = saved_count + 1
                q['frame_count'] = frame_count + 1

                if (q['saved_count']+1) >= bigbuf.shape[0]:
                    # cannot save more frames, save what we have
                    initiate_stop = True
        if initiate_stop:
            self.initiate_stop_saving()
                
        return draw_points, draw_linesegs
