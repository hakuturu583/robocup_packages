import dlib

if __name__ == "__main__":
    TrainingXml = "goal/training.xml"
    SvmFile = "goal/detector.svm"
    options = dlib.simple_object_detector_training_options()
    options.add_left_right_image_flips = True
    options.C = 5
    options.num_threads = 4
    options.be_verbose = True
    dlib.train_simple_object_detector(TrainingXml, SvmFile, options)
