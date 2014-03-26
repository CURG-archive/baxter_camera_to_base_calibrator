import tf
import tf_conversions
import rospy
import rosbag
import pdb
import numpy
import scipy
import scipy.optimize
run_pdb = False


def init_node():
    def is_uninitialized():
        return rospy.get_name() == '/unnamed'

    if is_uninitialized():
        rospy.init_node('calibrate_fiducial')
        

def matrix_to_parameter_descriptor(mat):
    return numpy.hstack([tf.transformations.euler_from_matrix(mat), tf.transformations.translation_from_matrix(mat)])

def parameter_descriptor_to_mat(param_desc):
    mat = tf.transformations.euler_matrix(*param_desc[:3])
    mat[:3,3] = param_desc[3:]
    return mat


def get_error(checkerboard_to_wrist_estimate, camera_in_body_estimate, checkerboard_in_camera_trans, wrist_in_body_trans):
    total_error = 0
    for checkerboard_in_camera, wrist_in_body in zip(checkerboard_in_camera_trans, wrist_in_body_trans):
        checkerboard_in_body = numpy.dot( numpy.linalg.inv(wrist_in_body),numpy.linalg.inv(checkerboard_to_wrist_estimate))
        
        #checkerboard_in_body = checkerboard_to_wrist_estimate * wrist_in_body
        camera_checkerboard_in_body = numpy.dot(numpy.linalg.inv(camera_in_body_estimate), numpy.linalg.inv(checkerboard_in_camera))
        #error = checkerboard_in_camera*camera_in_body_estimate - checkerboard_in_body
        error = camera_checkerboard_in_body - checkerboard_in_body
        total_error += sum(sum(abs(error)))
        if run_pdb:
            pdb.set_trace()
    return total_error

def get_total_error(parameters, checkerboard_in_camera_trans, wrist_in_body_trans):    
    arm_to_fiducial = parameter_descriptor_to_mat(parameters[:6])
    kinect_to_base = parameter_descriptor_to_mat(parameters[6:])
    error_norm = get_error(arm_to_fiducial, kinect_to_base, checkerboard_in_camera_trans, wrist_in_body_trans)
    print error_norm
    return numpy.sqrt(error_norm)



def get_transform_lists(bag_file_name):
    init_node()
    broadcaster = tf.TransformBroadcaster()
    listener = tf.TransformListener()
    listener.setUsingDedicatedThread(True)
    checkerboard_in_camera_trans = []
    wrist_in_body_trans = []
    pdb.set_trace()
    camera_in_body_estimate = None
    checkerboard_to_wrist_estimate = None
    bag = rosbag.Bag(bag_file_name)
    for topic, message, time in bag.read_messages('/tf'):
        for tf_message_stamped in message.transforms:
            tf_message = tf_message_stamped.transform
            translation = [tf_message.translation.x, tf_message.translation.y, tf_message.translation.z]
            rotation = [tf_message.rotation.x, tf_message.rotation.y, tf_message.rotation.z, tf_message.rotation.w]
            broadcaster.sendTransform( translation, rotation, rospy.Time.now(), tf_message_stamped.child_frame_id, tf_message_stamped.header.frame_id)
            tf_message_stamped.header.stamp = rospy.Time.now()
            listener.setTransform(tf_message_stamped,"user")
        for tf_message in message.transforms:
            if tf_message.header.frame_id == '/wrist_board' or tf_message.child_frame_id == '/wrist_board':
                if camera_in_body_estimate is None:
                    try:
                        listener.waitForTransform('/camera_link','/world',rospy.Time(0), rospy.Duration(.01))
                        camera_in_body_tf = listener.lookupTransform('/camera_link','/world',rospy.Time(0))
                        
                        print "got camera to world transform"
                        camera_in_body_estimate = tf_conversions.toMatrix(tf_conversions.fromTf(camera_in_body_tf))
                    except:
                        continue
                    print "got camera to world estimate"
                if checkerboard_to_wrist_estimate == None:
                    try:

                        listener.waitForTransform('/wrist_board','/left_wrist',rospy.Time(0), rospy.Duration(.01))
                        #(trans, rot) = listener.lookupTransform('/wrist_board','/left_wrist',rospy.Time(0))
                        #print trans
                        #print rot
                        checkerboard_to_wrist_tf = listener.lookupTransform('/wrist_board','/left_wrist',rospy.Time(0))
                        #print checkerboard_to_wrist_tf
                        #raw_input("press a key")

                        #checkerboard_to_wrist_tf = ((1.75, -0.75, -0.121),(-0.09, -0.04, 0.73, 0.66))
                        #print 'yinxiao test'

                        print "got wristboad in wrist"
                        checkerboard_to_wrist_estimate = tf_conversions.toMatrix(tf_conversions.fromTf(checkerboard_to_wrist_tf))
                        
                    except:
                        continue
                    print "got wristboard in wrist estimate"
                try:
                    listener.waitForTransform('/wrist_board','/camera_link',rospy.Time(0), rospy.Duration(.01))
                    listener.waitForTransform('/left_wrist','/world',rospy.Time(0), rospy.Duration(.1))

                    checkerboard_tf = listener.lookupTransform('/wrist_board','/camera_link',rospy.Time(0))
                    #print "got wristboard in camera"

                    checkerboard_in_camera_trans.append(tf_conversions.toMatrix(tf_conversions.fromTf(checkerboard_tf)))
                    
                    #print "got left wrist in world"
                    wrist_in_body_tf = listener.lookupTransform('/left_wrist','/world',rospy.Time(0))
                    wrist_in_body_trans.append(tf_conversions.toMatrix(tf_conversions.fromTf(wrist_in_body_tf)))
                except:
                    continue
                #print "finished loop"
                    

    return checkerboard_in_camera_trans, wrist_in_body_trans, camera_in_body_estimate, checkerboard_to_wrist_estimate
                
                


def estimate_kinect_to_base(bag_file_name):
    numpy_file_name = bag_file_name.split('.')[0]
    checkerboard_in_camera_trans= []
    wrist_in_body_trans = []
    camera_in_body_estimate = [] 
    checkerboard_to_wrist_estimate = []
    
    try:
        npzfile = numpy.load(numpy_file_name + '.npz')
        checkerboard_in_camera_trans = npzfile['checkerboard_in_camera_trans']
        wrist_in_body_trans = npzfile['wrist_in_body_trans']
        camera_in_body_estimate = npzfile['camera_in_body_estimate']
        checkerboard_to_wrist_estimate = npzfile['checkerboard_to_wrist_estimate']
        print 'loaded data from file'
    except:
        checkerboard_in_camera_trans, wrist_in_body_trans, camera_in_body_estimate, checkerboard_to_wrist_estimate = get_transform_lists(bag_file_name)
        numpy.savez(numpy_file_name, checkerboard_in_camera_trans = checkerboard_in_camera_trans, wrist_in_body_trans = wrist_in_body_trans, camera_in_body_estimate = camera_in_body_estimate, checkerboard_to_wrist_estimate = checkerboard_to_wrist_estimate) 

    
    checkerboard_tran_params = matrix_to_parameter_descriptor(checkerboard_to_wrist_estimate)
    camera_tran_params = matrix_to_parameter_descriptor(camera_in_body_estimate)
    parameters = numpy.hstack([checkerboard_tran_params, camera_tran_params])    
    retval = scipy.optimize.fmin_powell(get_total_error, parameters, args=(checkerboard_in_camera_trans, wrist_in_body_trans), xtol=0.001, ftol=0.001, maxiter=10000, maxfun=10000, full_output=0, disp=1, retall=0, callback=None, direc=None)
    kinect_to_base = parameter_descriptor_to_mat(retval[6:])
    return  tf.transformations.translation_from_matrix(kinect_to_base), tf.transformations.quaternion_from_matrix(kinect_to_base)

def main():
    estimate_kinect_to_base('data/2014-03-21-11-52-58.bag')

if __name__ == "__main__":
    main()
