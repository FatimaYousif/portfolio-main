import ProjectCard from "@/components/ProjectCard"
import RevealContent from "@/components/ReavealContent"
import SectionBadge from "@/components/SectionBadge"

function Projects() {
  return (
    <>
      <RevealContent>
        <>
          <SectionBadge sectionName={""} />
          <h2 className="text-3xl font-extrabold text-center mb-6 md:mb-10">My Projects</h2>
        </>
      </RevealContent>
      <div className="flex flex-col gap-12 lg:gap-20">


        <ProjectCard
          name={"Sim2Real: Controlling a Swarm of Crazyflies using Reynolds Rules and Consensus Protocol"}
          description={"This project implements swarm control for Crazyflies UAVs using Reynolds Rules for flocking and a Consensus Protocol for coordinated movement. It integrates rendezvous and formation control in ROS2 and Gazebo, enabling agents to converge and maintain geometric formations. Tested in both simulation and real-world environments, the system demonstrates adaptability and scalability, with results highlighting the impact of communication topologies on swarm dynamics."}
          // technologies={["NextJS", "React.JS", "Tailwind CSS", "API", "Responsive", "Github Actions"]}
          technologies={[]}
          
          imageSrc={"/images/mrs.gif"}
          imagePosition={"right"}
          githubLink={"https://drive.google.com/file/d/1Z6rUHu_9CV6AMe3nXfcD9oOo7O9E0dog/view?usp=sharing"}
          // directLink={"https://la-data-verte.vercel.app/"}
        />

        
          <ProjectCard
          name={"Stereo Visual-Odometry (VO) on the KITTI Dataset"}
          description={"This projects contains the implementation of Stereo VO pipeline in Python on the KITTI dataset. It processes stereo image data using SIFT, feature matching using BFMatcher, triangulation of points, to estimate the motion of a camera (w.r.t its starting position) in 3D space using the approach of minimizing the 3D to 2D reprojection error with PnP and RANSAC."}
          // technologies={["NextJS", "React.JS", "Tailwind CSS", "API", "Responsive", "Github Actions"]}
          technologies={[]}
          
          imageSrc={"/images/recording.gif"}
          imagePosition={"right"}
          githubLink={"https://github.com/FatimaYousif/Stereo_VO"}
          // directLink={"https://la-data-verte.vercel.app/"}
        />

        <ProjectCard
          name={"Aerial Robotics"}
          description={"In this Aerial Robotics course, lab work included design and implementations of attitude control of a quadrotor, cascade control of a single quadrotor axis in MATLAB, cascade horizontal control of quadrotor in the Gazebo simulator and on the real DJI Tello quadrotor. "}
          // technologies={["NextJS", "React.JS", "Tailwind CSS", "API", "Responsive", "Github Actions"]}
          technologies={[]}
          
          imageSrc={"/images/ar.gif"}
          imagePosition={"left"}
          githubLink={"https://github.com/FatimaYousif/AerialRobotics"}
          // directLink={"https://la-data-verte.vercel.app/"}
        />

        <ProjectCard
          name={"Deep Learning"}
          description={"In this Deep Learning course lab work, PyTorch implementations included working on logistic regression and gradient descent, implementing fully connected models on the MNIST dataset, Convolutional models for image classification tasks on MNIST and CIFAR, Recurrent models for analysis of sentiment classification with the Stanford Sentiment Treebank (SST) dataset followed by detailed implementations on metric embeddings."}
          // technologies={["NextJS", "React.JS", "Tailwind CSS", "API", "Responsive", "Github Actions"]}
          technologies={[]}
          
          imageSrc={"/images/dl.png"}
          imagePosition={"right"}
          githubLink={"https://github.com/FatimaYousif/DeepLearning"}
          // directLink={"https://la-data-verte.vercel.app/"}
        />

      <ProjectCard
          name={"Human Detection and Tracking"}
          description={"This project focuses on human detection and tracking using the state-of-the-art YOLOv9 object detection model and the DeepSORT multi-object tracking algorithm. The methodology integrates Kalman filtering for motion prediction and deep learning-based appearance matching. The system is tested under various conditions, addressing challenges such as occlusions, identity switches, and tracking interruptions. "}
          // technologies={["NextJS", "React.JS", "Tailwind CSS", "API", "Responsive", "Github Actions"]}
          technologies={[]}
          
          imageSrc={"/images/hri.png"}
          imagePosition={"left"}
          githubLink={"https://drive.google.com/file/d/1InWfWw6GBx91oDSp3u7YZ171ZZodPn7l/view?usp=sharing"}
          // directLink={"https://la-data-verte.vercel.app/"}
        />




        <ProjectCard
          name={"Frontier Based Exploration Using Kobuki Turtlebot"}
          description={"Frontier exploration project using an RGB-D camera mounted on a Kobuki Turtlebot. The project integrates advanced path planning techniques, combining the RRT* algorithm with Dubin’s path to map unknown environments. Additionally, a hybrid control system, which merges PID control with principles from the Pure Pursuit Controller used to optimize the robot’s velocity profiles. The implementation is done in Python within ROS framework, with simulation testing performed in the Stonefish simulator before real-world testing."}
          // technologies={["NextJS", "React.JS", "Tailwind CSS", "API", "Responsive", "Github Actions"]}
          technologies={[]}
          
          imageSrc={"/images/hol_hop.png"}
          imagePosition={"left"}
          githubLink={"https://drive.google.com/file/d/1NlvOoZAyUrwlc5uZDKQOGGCkYU58ea0j/view?usp=drive_link"}
          // directLink={"https://la-data-verte.vercel.app/"}
        />


        <ProjectCard
          name={"Monocular Visual Odometry for an Underwater Vehicle"}
          description={"Monocular visual odometry (VO) for an Autonomous Underwater Vehicle (AUV) through an integrated approach combining extended Kalman filter (EKF) based navigation. The methodology employs SIFT feature detection and FLANN matching to process images from a ROSBag. A key contribution of this work is the incorporation of EKF to provide a refined estimation of the vehicle´s motion and trajectory."}
          // technologies={["NextJS", "React.JS", "Tailwind CSS", "API", "Responsive", "Github Actions"]}
          technologies={[]}
          
          imageSrc={"/images/hope.gif"}
          imagePosition={"right"}
          githubLink={"https://drive.google.com/file/d/1tw3GywQTFNa43fAUBPzNxo-abeP9HBu0/view?usp=drive_link"}
          // directLink={"https://la-data-verte.vercel.app/"}
        />
        
        <ProjectCard
          name={"Pose Based SLAM using the Extended Kalman Filter on a Kobuki Turtlebot"}
          description={"Pose based EKF SLAM algorithm using the Extended Kalman Filter (EKF), incorporating view poses where environmental scans are integrated into the state vector. This algorithm was evaluated through both simulation and real-world testing."}
          // technologies={["NextJS", "React.JS", "Tailwind CSS", "API", "Responsive", "Github Actions"]}
          technologies={[]}
          
          imageSrc={"/images/hol_hop.png"}
          imagePosition={"left"}
          githubLink={"https://drive.google.com/file/d/19PtEXhWmLMEDCZ_8OUZyceBBa_9b1kHx/view?usp=drive_link"}
          // directLink={"https://la-data-verte.vercel.app/"}
        />


        <ProjectCard
          name={"Kinematic Control System for a Mobile Manipulator, based on the Task-Priority Redundancy Resolution Algorithm"}
          description={"Kinematic control system derived and implemented on a differential-drive robot (Kobuki Turtlebot 2), fitted with a 4 DOF manipulator (uFactory uArm Swift Pro). The system is predicated on the task-priority redundancy resolution algorithm. The implementation is done using ROS and the Stonefish simulator."}
          // technologies={["NextJS", "React.JS", "Tailwind CSS", "API", "Responsive", "Github Actions"]}
          technologies={[]}
          
          imageSrc={"/images/hoi.png"}
          imagePosition={"right"}
          githubLink={"https://drive.google.com/file/d/1AjLQ44gIUkyi49Ndg9Sg08PuGjYsaxSl/view?usp=drive_link"}
          // directLink={"https://la-data-verte.vercel.app/"}
        />


        <ProjectCard
          name={"ROS2 Collision Avoidance Using Cross and Direct Echo of Bosch Ultrasonic Sensor Systems"}
          description={"Testing and comparing of two ultrasonic sensors i.e. Bosch and Valeo for the obstacle avoidance task to include the safety braking feature (setting thresholds to slow down or stop the robot for collision avoidance with ROS2) which involved performing multiple field tests of different high grass."}
          // technologies={["NextJS", "React.JS", "Tailwind CSS", "API", "Responsive", "Github Actions"]}
          technologies={[]}
          
          imageSrc={"/images/paltech.png"}
          imagePosition={"right"}
          githubLink={"https://github.com/FatimaPaltech/Bosch_uss_ws"}
          // directLink={"https://la-data-verte.vercel.app/"}
        />



        {/* <ProjectCard
          name={"Turtlebot Online Path Planning"}
          description={"Online path planning for a Turtlebot (ground robot) using RRT with smoothing algorithm for collision-free path planning and obstacle avoidance in ROS, Gazebo and RViz."}
          // technologies={["Next JS", "React.JS", "Tailwind CSS", "Framer motion", "Responsive", "Shadcn"]}
          technologies={[]}
          imageSrc={"/images/turtlebot_pp.gif"}
          imagePosition={"left"}
          githubLink={"https://github.com/FatimaYousif/turtlebot_online_path_planning"}
          // directLink={"https://antoinedangleterre.com"}
        /> */}

        <ProjectCard
          name={"SLAM - Differential Drive Mobile Robot "}
          description={"Simultaneous Localization and Mapping(SLAM) algorithms for a differential drive mobile robot with python simulations and plotting."}
          // technologies={["NextJS", "React.JS", "Tailwind CSS", "API", "Responsive", "Github Actions"]}
          technologies={[]}
          
          imageSrc={"/images/SLAM.gif"}
          imagePosition={"left"}
          githubLink={"https://github.com/FatimaYousif/Feature_EKF_SLAM"}
          // directLink={"https://la-data-verte.vercel.app/"}
        />

        <ProjectCard
          name={"Behaviour trees for pick-place of objects "}
          description={"Used the py_trees library followed by the results tested with turtlebot simulations in RViz with different complex environments having path planning and obstacle avoidance involved."}
          // technologies={["React.JS", "Tailwind CSS", "Responsive"]}
          technologies={[]}
          
          imageSrc={"/images/BT_stage3.gif"}
          imagePosition={"right"}
          githubLink={"https://github.com/FatimaYousif/pick_up_objects_task/"}
        />
        
        {/* <ProjectCard
          name={"Mobile Manipulator Task-Priority Kinematic Control "}
          description={"Task-priority redundancy resolution control algorithm for the 6 DOF manipulator with mobile base using python simulations."}
          // technologies={["Blog", "SEO", "Marketing", "Webdesign", "UX/UI", "WordPress", "Maintenance"]}
          technologies={[]}
          
          imageSrc={"/images/tp.gif"}
          imagePosition={"left"}
          githubLink={"https://github.com/FatimaYousif/Hands_on_Intervention/tree/main/mobile-manipulator"}
        
          // directLink={"https://kodiz.fr"}
        /> */}


        <ProjectCard
          name={"Pick and Place Application with the Staubli TS60 and TX60 Robot"}
          description={"Worked on industrial manipulators - TS60 and TX60 robots for classification, assembling pieces and performing pick-place tasks on simulation alongside real-robot implementation."}
          // technologies={["React.JS", "Tailwind CSS", "Responsive"]}
          technologies={[]}
          
          imageSrc={"/images/TSX.png"}
          imagePosition={"right"}
          githubLink={"https://github.com/FatimaYousif/Robot_Manipulation"}
        />
      <ProjectCard
          name={"Palletizing Application with UR3e Collaborative Robot (CoBot)"}
          description={"Worked with the collaborative robot by developing a pick-and-place program for pallets to perform palletizing application by utilizing industrial UR3e Collaborative Robot."}
          // technologies={["Blog", "SEO", "Marketing", "Webdesign", "UX/UI", "WordPress", "Maintenance"]}
          technologies={[]}
          
          imageSrc={"/images/cobot.png"}
          imagePosition={"left"}
          // directLink={"https://kodiz.fr"}
          githubLink={"https://github.com/FatimaYousif/Robot_Manipulation/tree/main/The%20UR3e%20Collaborative%20Robot"}
        />
        <ProjectCard
          name={"Stereo Visual Odometry (VO) for Grizzly Robotic Utility Vehicle"}
          description={"Developed VO pipeline from stereo camera calibration, feature extraction, and matching using SURF features and utilizing bucketing strategies and circular matching for accurate apparent motion computation and effective noise/outlier rejection, Structure from motion (2D-to-2D, 3D-to-2D, and 3D-to-3D) for triangulation and refinement using bundle adjustment. The final VO trajectory was also extensively compared with GPS-generated ground truth data."}
          // technologies={["React.JS", "Tailwind CSS", "Responsive"]}
          technologies={[]}
          
          imageSrc={"/images/SVO.png"}
          imagePosition={"right"}
          githubLink={"https://github.com/FatimaYousif/Multi_View_Geometry/tree/main/stereo_visual_odometry"}
        />
        
        <ProjectCard
          name={"Event Based Cameras (EBC) "}
          description={"Worked on Event-based EBCs examining event data alongside ground truth using Davis using the frame-based approach for encoding raw event-based data into frames compatible with CNNs and RNNs and also applied Motion Compensation."}
          // technologies={["Blog", "SEO", "Marketing", "Webdesign", "UX/UI", "WordPress", "Maintenance"]}
          technologies={[]}
          
          imageSrc={"/images/EBC.png"}
          imagePosition={"left"}
          // directLink={"https://kodiz.fr"}
          githubLink={"https://github.com/FatimaYousif/Event_based_Cameras"}
        
        />
      
        <ProjectCard
          name={"Machine Vision Projects "}
          description={"Contributed to projects such as Augmented Reality, Camera Calibration, Detecting Aruco markers, and Generating Fiducial Makers with computer vision, and image processing in C++. "}
          // technologies={["Blog", "SEO", "Marketing", "Webdesign", "UX/UI", "WordPress", "Maintenance"]}
          technologies={[]}
          
          imageSrc={"/images/mv.png"}
          imagePosition={"right"}
          // directLink={"https://kodiz.fr"}
          githubLink={"https://github.com/FatimaYousif/Machine_Vision_Projects"}
        
        />

        
        {/* <ProjectCard
          name={" Planning Domain Definition Language(PDDL) "}
          description={"Worked on PDDL modelling (planning domains and problem) as a probabilistic planning language and planning based approach for the container stacking problem. "}
          // technologies={["Blog", "SEO", "Marketing", "Webdesign", "UX/UI", "WordPress", "Maintenance"]}
          technologies={[]}
          
          imageSrc={"/images/pddl.png"}
          imagePosition={"left"}
          // directLink={"https://kodiz.fr"}
          githubLink={"https://github.com/FatimaYousif/PDDL_modelling_Hands_On_2"}
        
        /> */}


        {/* <ProjectCard
          name={"Path Planners "}
          description={"Developed 2D path planners including; RRT, RRT* and A star tested on a grid map in python."}
          // technologies={["Blog", "SEO", "Marketing", "Webdesign", "UX/UI", "WordPress", "Maintenance"]}
          technologies={[]}
          
          imageSrc={"/images/planners.png"}
          imagePosition={"right"}
          // directLink={"https://kodiz.fr"}
          githubLink={"https://github.com/FatimaYousif/Autonomous_Systems"}
        
        />     */}

        
        {/* <ProjectCard
          name={"Machine Learning Projects "}
          description={"Developed Projects on machine learning topics such as logistic regression, decision trees, gradient descent for linear regression, support vector machines (SVM), fully connected networks (FCN), and convolutional neural networks (CNN)."}
          // technologies={["Blog", "SEO", "Marketing", "Webdesign", "UX/UI", "WordPress", "Maintenance"]}
          technologies={[]}
          
          imageSrc={"/images/ml.png"}
          imagePosition={"left"}
          // directLink={"https://kodiz.fr"}
          githubLink={"https://github.com/FatimaYousif/Machine_Learning_Projects"}
        
        />     */}

        
        <ProjectCard
          name={"Reinforcement Learning-Based Path Planning for Autonomous Robots in Static Environments "}
          description={"Implemented the Q-learning algorithm on a point (omnidirectional) robot for path planning and navigation purposes."}
          // technologies={["Blog", "SEO", "Marketing", "Webdesign", "UX/UI", "WordPress", "Maintenance"]}
          technologies={[]}
          
          imageSrc={"/images/qlearning.gif"}
          imagePosition={"right"}
          // directLink={"https://kodiz.fr"}
          githubLink={"https://github.com/FatimaYousif/Autonomous_Systems/blob/main/QLearning.ipynb"}
        
        />    


        <ProjectCard
          name={"Image Captioning Deep Learning Model "}
          description={"Developed this Final Year Project by using cutting-edge including Deep Learning, Computer Vision, and data mining technologies including Keras libraries with Flask in the backend and AWS."}
          // technologies={["Blog", "SEO", "Marketing", "Webdesign", "UX/UI", "WordPress", "Maintenance"]}
          technologies={[]}
          
          imageSrc={"/images/imgcap.gif"}
          imagePosition={"left"}
          // directLink={"https://kodiz.fr"}
          githubLink={"https://github.com/FatimaYousif/Image-Captioning-Deep-Learning-Model"}
        
        />    



      </div>
    </>
  )
}

export default Projects