"use client"

import { useState, useMemo } from "react"
import ProjectCard from "@/components/ProjectCard"
import RevealContent from "@/components/ReavealContent"
import SectionBadge from "@/components/SectionBadge"

// Order controls the order the filter buttons appear in
const CATEGORIES = [
  "All",
  "Robotics",
  "Multi-Robot Systems",
  "Control",
  "Reinforcement Learning",
  "Computer Vision",
  "Industrial Robots",
  "LLMs",
  "Neuromorphic Vision",
]

const projectsData = [
  {
    name: "Vision-Based Tracking and Following of a Moving Target Using an Unmanned Aerial Vehicle",
    description:
      "Developed a UAV perception and control pipeline integrating RGB vision with autonomous drone control. Implemented multi-object tracking using YOLO-based detectors and state-of-the-art trackers (BoT-SORT and ByteTrack), incorporating IoU-based association and Kalman filtering to improve tracking under occlusions. The system was developed in ROS 2 and evaluated in both simulation and real-world experiments on a Holybro X500 UAV with a Pixhawk 4 flight controller, enabling real-time onboard AI inference.",
    categories: ["Robotics", "Computer Vision"],
    technologies: [],
    imageSrc: "/images/project.gif",
    imagePosition: "right",
    directLink: "https://drive.google.com/file/d/15_ZbHtW5GLerMhS5jtIts86WxeXCNYkw/view?usp=drive_link",
  },
  {
    name: "LiDAR-Camera Sensor Fusion - KITTI 3D Object Detection",
    description:
      "Fuses YOLO-based 2D object detection with raw LiDAR point clouds to recover 3D depth for detected objects. Implements the full KITTI calibration chain (LiDAR → Camera → IMU → Geodetic), projecting point clouds onto camera images with RANSAC ground-plane removal and associating detections with LiDAR depth via nearest-neighbor matching. Outputs real-world GPS-located detections.",
    categories: ["Computer Vision", "Robotics"],
    technologies: [],
    imageSrc: "/images/fusion.png",
    imagePosition: "right",
    githubLink: "https://github.com/FatimaYousif/CameraLidar_fusion_3DObjectDetection",
  },
  {
    name: "Decentralized UAV Swarm Control using Reynolds Flocking and Consensus Protocol",
    description:
      "This project implements swarm control for Crazyflies UAVs using Reynolds Rules for flocking and a Consensus Protocol for coordinated movement. It integrates rendezvous and formation control in ROS2 and Gazebo, enabling agents to converge and maintain geometric formations. Tested in both simulation and real-world environments, the system demonstrates adaptability and scalability.",
    categories: ["Multi-Robot Systems", "Robotics"],
    technologies: [],
    imageSrc: "/images/mrs.gif",
    imagePosition: "right",
    directLink: "https://drive.google.com/file/d/1gidcwXDXMhnHQ3EM0YPyVH-mxWnaW1zA/view",
  },
  {
    name: "Frontier Based Exploration Using Kobuki Turtlebot",
    description:
      "Using RGB-D camera mounted on a Kobuki Turtlebot, the project integrates advanced path planning techniques, combining the RRT* algorithm with Dubins path to map unknown environments with the primary objective of enabling the Turtlebot to autonomously explore unknown environments by identifying and navigating to frontiers. A hybrid control system, which merges PID control with principles from the Pure Pursuit Controller is used. Validated both in simulation and real world.",
    categories: ["Robotics"],
    technologies: [],
    imageSrc: "/images/hol_hop.png",
    imagePosition: "right",
    directLink: "https://drive.google.com/file/d/1NlvOoZAyUrwlc5uZDKQOGGCkYU58ea0j/view?usp=drive_link",
  },
  {
    name: "Monocular Visual Odometry for an Autonomous Underwater Vehicle (AUV)",
    description:
      "M-VO for an AUV through an integrated approach combining extended Kalman filter (EKF) based navigation. The methodology employs SIFT feature detection and FLANN matching to (offline / post) process images from a ROSBag. A key contribution of this work is the incorporation of EKF to provide a refined estimation of the vehicle´s motion and trajectory.",
    categories: ["Robotics", "Computer Vision"],
    technologies: [],
    imageSrc: "/images/hope.gif",
    imagePosition: "right",
    directLink: "https://drive.google.com/file/d/1tw3GywQTFNa43fAUBPzNxo-abeP9HBu0/view?usp=drive_link",
  },
  {
    name: "Behaviour Trees for Pick and Place of Objects ",
    description:
      "Implemented behavior trees using the py_trees library and validated the approach through TurtleBot simulations in RViz, demonstrating path planning and obstacle avoidance across various complex environments.",
    categories: ["Robotics"],
    technologies: [],
    imageSrc: "/images/BT_stage3.gif",
    imagePosition: "right",
    githubLink: "https://github.com/FatimaYousif/pick_up_objects_task/",
  },
  {
    name: "Stereo Visual Odometry on the KITTI Dataset",
    description:
      " Implementation of Stereo VO pipeline in Python on the KITTI dataset. It processes stereo image data using SIFT, feature matching using BFMatcher, triangulation of points, to estimate the motion of a camera (w.r.t its starting position) in 3D space using the approach of minimizing the 3D to 2D reprojection error with PnP and RANSAC.",
    categories: ["Computer Vision"],
    technologies: [],
    imageSrc: "/images/recording.gif",
    imagePosition: "right",
    githubLink: "https://github.com/FatimaYousif/Stereo_VO",
  },
  {
    name: "Modeling and Control of Quadrotor UAVs",
    description:
      "In this Aerial Robotics course, lab work included design and implementations of attitude control of a quadrotor, cascade control of a single quadrotor axis in MATLAB, cascade horizontal control of quadrotor in the Gazebo simulator and on the real DJI Tello quadrotor. ",
    categories: ["Robotics", "Control"],
    technologies: [],
    imageSrc: "/images/ar.gif",
    imagePosition: "right",
    githubLink: "https://github.com/FatimaYousif/AerialRobotics",
  },
  {
    name: "Deep Learning",
    description:
      "In this Deep Learning course lab work, PyTorch implementations included working on logistic regression and gradient descent, implementing fully connected models on the MNIST dataset, Convolutional models for image classification tasks on MNIST and CIFAR, Recurrent models for analysis of sentiment classification with the Stanford Sentiment Treebank (SST) dataset followed by detailed implementations on metric embeddings.",
    categories: ["Computer Vision", "LLMs"],
    technologies: [],
    imageSrc: "/images/dl.png",
    imagePosition: "right",
    githubLink: "https://github.com/FatimaYousif/DeepLearning",
  },
  {
    name: "Pose Based SLAM using the Extended Kalman Filter (EKF) on a Kobuki Turtlebot",
    description:
      "Implemented a Pose-Based P- EKF SLAM system integrating IMU and 2D LiDAR data on a Kobuki Turtlebot. The algorithm maintains robot pose history for map building and localization, using ICP for scan matching and robust state updates. Validated in both simulated (Stonefish) and real-world environments, PEKFSLAM demonstrated superior accuracy and stability compared to conventional EKF-based SLAM approaches.",
    categories: ["Robotics"],
    technologies: [],
    imageSrc: "/images/hol_hop.png",
    imagePosition: "right",
    directLink: "https://drive.google.com/file/d/19PtEXhWmLMEDCZ_8OUZyceBBa_9b1kHx/view?usp=drive_link",
  },
  {
    name: "Kinematic Control System for a Mobile Manipulator, based on the Task-Priority Redundancy Resolution Algorithm",
    description:
      "Designed and implemented a kinematic control system for a mobile manipulator (Kobuki Turtlebot 2 with a 4-DOF uArm Swift Pro), using a task-priority redundancy resolution algorithm. Developed in ROS and tested in the Stonefish simulator, the system performed complex pick-and-place tasks, including ArUco marker-based navigation.",
    categories: ["Robotics", "Control", "Industrial Robots"],
    technologies: [],
    imageSrc: "/images/hoi.png",
    imagePosition: "right",
    directLink: "https://drive.google.com/file/d/1AjLQ44gIUkyi49Ndg9Sg08PuGjYsaxSl/view?usp=drive_link",
  },
  {
    name: "SLAM - Differential Drive Mobile Robot ",
    description:
      "Simultaneous Localization and Mapping (SLAM) algorithms for a differential drive mobile robot with python simulations and plotting.",
    categories: ["Robotics"],
    technologies: [],
    imageSrc: "/images/SLAM.gif",
    imagePosition: "right",
    githubLink: "https://github.com/FatimaYousif/Feature_EKF_SLAM",
  },
  {
    name: "Pick and Place Application with the Staubli TS60 and TX60 Robot",
    description:
      "Worked on industrial manipulators - TS60 and TX60 robots for classification, assembling pieces and performing pick-place tasks on simulation alongside real-robot implementation.",
    categories: ["Industrial Robots"],
    technologies: [],
    imageSrc: "/images/TSX.png",
    imagePosition: "right",
    githubLink: "https://github.com/FatimaYousif/Robot_Manipulation",
  },
  {
    name: "Palletizing Application with UR3e Collaborative Robot (CoBot)",
    description:
      "Worked with the 6 DOF CoBot to develop a pick-and-place program with pallets to achieve simple palletizing application. Testing was conducted collaboratively in the laboratory.",
    categories: ["Industrial Robots"],
    technologies: [],
    imageSrc: "/images/cobot.png",
    imagePosition: "right",
    githubLink:
      "https://github.com/FatimaYousif/Robot_Manipulation/tree/main/The%20UR3e%20Collaborative%20Robot",
  },
  {
    name: "Stereo Visual Odometry (VO) for Grizzly Robotic Utility Vehicle",
    description:
      "Developed VO pipeline from stereo camera calibration, feature extraction, and matching using SURF features and utilizing bucketing strategies and circular matching for accurate apparent motion computation and effective noise/outlier rejection, Structure from motion (2D-to-2D, 3D-to-2D, and 3D-to-3D) for triangulation and refinement using bundle adjustment. The final VO trajectory was also extensively compared with GPS-generated ground truth data.",
    categories: ["Computer Vision", "Robotics"],
    technologies: [],
    imageSrc: "/images/SVO.png",
    imagePosition: "right",
    githubLink: "https://github.com/FatimaYousif/Multi_View_Geometry/tree/main/stereo_visual_odometry",
  },
  {
    name: "Event Based Cameras (EBC) ",
    description:
      "Analyzed EBC data by comparing it with ground truth using DAVIS. Employed a frame-based approach to convert raw event data into frames suitable for CNNs and RNNs, and applied motion compensation techniques.",
    categories: ["Neuromorphic Vision"],
    technologies: [],
    imageSrc: "/images/EBC.png",
    imagePosition: "right",
    githubLink: "https://github.com/FatimaYousif/Event_based_Cameras",
  },
  {
    name: "Machine Vision Projects ",
    description:
      "Worked on tasks such as Augmented Reality, Camera Calibration, Detecting Aruco markers, and Generating Fiducial Makers with computer vision, and image processing in CPP.",
    categories: ["Computer Vision"],
    technologies: [],
    imageSrc: "/images/mv.png",
    imagePosition: "right",
    githubLink: "https://github.com/FatimaYousif/Machine_Vision_Projects",
  },
  {
    name: "Reinforcement Learning-Based Path Planning for Autonomous Robots in Static Environments ",
    description:
      "Implemented the Q-learning algorithm on a point (omnidirectional) robot for path planning and navigation purposes.",
    categories: ["Reinforcement Learning"],
    technologies: [],
    imageSrc: "/images/qlearning.gif",
    imagePosition: "right",
    githubLink: "https://github.com/FatimaYousif/Autonomous_Systems/blob/main/QLearning.ipynb",
  },
  {
    name: "Image Captioning Deep Learning Model ",
    description:
      "In this undergraduate research project, developed an attention-based CNN-RNN image captioning system that converts visual content into natural-language descriptions. The model uses InceptionV3 as the encoder and GRU as the decoder, trained on datasets including MS COCO, Flickr8k, and Flickr30k.",
    categories: ["Computer Vision", "LLMs"],
    technologies: [],
    imageSrc: "/images/imgcap.gif",
    imagePosition: "right",
    directLink: "https://drive.google.com/file/d/1xL6GDLfT-wirNT5IENKNfWQAzqt80B2Q/view?usp=sharing",
  },
]

function Projects() {
  const [activeCategory, setActiveCategory] = useState("All")

  const filteredProjects = useMemo(() => {
    if (activeCategory === "All") return projectsData
    return projectsData.filter((p) => p.categories.includes(activeCategory))
  }, [activeCategory])

  return (
    <>
      <RevealContent>
        <>
          <SectionBadge sectionName={""} />
          <h2 className="text-3xl font-extrabold text-center mb-6 md:mb-10">Projects</h2>
        </>
      </RevealContent>

      {/* Category filter bar */}

      <div className="flex flex-wrap justify-center gap-2 mb-10">
      {CATEGORIES.map((cat) => (
        <button
          key={cat}
          onClick={() => setActiveCategory(cat)}
          className="px-4 py-2 rounded-md text-sm font-medium transition-colors duration-200 text-white"
          style={{
            backgroundColor: activeCategory === cat ? "#1e4066" : "#2a5c95",
          }}
          onMouseEnter={(e) => (e.currentTarget.style.backgroundColor = "#234b7a")}
          onMouseLeave={(e) =>
            (e.currentTarget.style.backgroundColor = activeCategory === cat ? "#1e4066" : "#2a5c95")
          }
        >
          {cat}
        </button>
      ))}
    </div>

      <div className="flex flex-col gap-12 lg:gap-20">
        {filteredProjects.map((project, idx) => (
          <ProjectCard
            key={project.name + idx}
            name={project.name}
            description={project.description}
            technologies={project.technologies}
            imageSrc={project.imageSrc}
            imagePosition={project.imagePosition}
            directLink={project.directLink}
            githubLink={project.githubLink}
          />
        ))}
      </div>
    </>
  )
}

export default Projects
