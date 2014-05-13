# Message participating in Nao face detection module.
# Extra information about the detected face (place of eyes, eyebrows etc)

std_msgs/Int32 face_id
std_msgs/Float32 score_reco
std_msgs/String face_label

nao_extras_msgs/FaceEyePoints left_eye_points
nao_extras_msgs/FaceEyePoints right_eye_points
nao_extras_msgs/FaceEyebrowPoints left_eyebrow_points
nao_extras_msgs/FaceEyebrowPoints right_eyebrow_points
nao_extras_msgs/FaceNosePoints nose_points
nao_extras_msgs/FaceMouthPoints mouth_points
