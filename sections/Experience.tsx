import SectionBadge from "@/components/SectionBadge"
import Experience from "@/components/Experience"
import RevealContent from "@/components/ReavealContent"

function Experiences() {
    return (

         <section id="experience" className="scroll-mt-24">
        <>
            <RevealContent>
                <>
                    <SectionBadge sectionName={""} />
                    <h2 className="text-3xl font-extrabold text-center mb-6 md:mb-10">Experiences</h2>
                </>
            </RevealContent>
            <div className="flex flex-col gap-12">

               <Experience post={"Robotics Researcher"} dateStart={"September 2025"} dateEnd={"August 2026"} company={"Saxion University of Applied Sciences - Smart Mechatronics and Robotics Research Group"}>
                    {/* <p> */}
                    {/* Contributing to the development of an autonomous security drone system for target detection, tracking, and following by integrating gimbal-based target tracking, RGB and thermal camera perception, and a search-and-follow framework combining randomized exploration and path planning, real-time target tracking, and fire detection/localization. */}
                    {/* Contributing to the development of an autonomous security drone for target detection, tracking, and following. Working with RGB and thermal cameras, gimbal-based tracking, path planning, and fire detection/localization, while integrating sensors, testing system performance, and troubleshooting field deployments. Collaborating with research and industry partners on experiments, system validation, technical documentation, and supervising student projects in autonomous systems. */}
                    {/* I continued developing and evaluating unmanned aerial vehicle (UAV)-based anomaly tracking on a physical quadrotor platform (Holybro X500 airframe and Pixhawk4 flight controller) equipped with an edge AI system. My work also included researching perception methods using RGB and thermal sensing modalities, and deploying a search-and-follow framework based on hierarchical state-machine control that integrates randomized exploration, real-time target tracking, and fire detection/localization for situational awareness. Working alongside industrial partners on this project taught me how to communicate across disciplines, integrate contributions from multiple work packages, and solve practical challenges that arise during field testing. */}
                    {/* <ul>
                        <li> Contributed to the development of an autonomous security drone for target detection, tracking, and following within a field robotics research project. </li>
                        <li> Researched perception methods using RGB and thermal sensing modalities, and a search-and-follow framework using hierarchical state-machine control that integrates randomized path planning, real-time target tracking, and fire detection/localization for situational awareness. </li>
                        <li> Collaborated with a multidisciplinary research team on sensor and system integration of different work packages, and scientific documentation. Additionally, supported field tests with industry partners through experiments. </li>
                        <li> Co-supervised undergraduate students working on an autonomous security drone project and served as the client representative for first-year electrical engineering course autonomous systems projects.</li>
                        
                    </ul> */}
                    {/* </p> */}
                <ul className="list-disc pl-6">
                    <li>Developed an autonomous security drone integrating planning, perception, localization, and control on a Jetson onboard a Holybro X500, 
                    validated in simulation and real-world field tests. A hierarchical state machine runs randomized patrol routes under battery and no-fly-zone constraints, 
                    switches to person-following or fire-tracking when
                    an anomaly is detected, and resumes patrol afterward. Contributed to project work
                    package deliverables, demonstrators, and scientific documentation.</li>
                    <li> Built an RGB and thermal anomaly dete ction pipeline (people, vehicles, fires) for day
                    and night drone operation: custom tiny-object aerial dataset (collection, annotation,
                    augmentation), YOLO fine-tuning evaluated with mAP and precision/recall, real-time
                    onboard inference on a Jetson, contour-based fire detection on 16-bit LWIR imagery,
                    and GPS localization of fires for a fire-extinguishing drone.</li>
                    <li> Co-supervised undergraduate students on the security drone project, alongside serving
                    as a client representative for first-year Electrical Engineering students’ Autonomous
                    Racing Car challenge (technical mentoring, research infrastructure, final competition).</li>
                    {/* <li>Contributed to the development of an autonomous security drone for target detection, tracking, and following within a field robotics research project.</li>
                    <li>Researched perception methods using RGB and thermal sensing modalities, and a search-and-follow framework using hierarchical state-machine control that integrates randomized path planning, real-time target tracking, and fire detection/localization for situational awareness.</li>
                    <li>Collaborated with a multidisciplinary research team on sensor and system integration of different work packages, and scientific documentation. Additionally, supported field tests with industrial partners.</li>
                    <li>Co-supervised undergraduate students working on an autonomous security drone project and served as the client representative for first-year electrical engineering course autonomous systems projects.</li> */}
                </ul>
                </Experience>
            
            <Experience post={"Master Thesis Intern"} dateStart={"March 2025"} dateEnd={"May 2025"} company={"Saxion University of Applied Sciences - Smart Mechatronics and Robotics Research Group"}>
                    {/* <p>
                    My research work on “Vision-based Tracking and Following of a Moving Target Using a UAV”, focused on challenges mainly in RGB modality for robust person detection and tracking, dealing with occlusions and identity switches, and real-time prediction in dynamic environments. The work integrates a hybrid system combining filtering-based tracking (Kalman Filter), vision-based deep learning models (YOLO), and adaptive control strategies. Overall, this research seeks to leverage situational-aware techniques to enable intelligent UAV deployment in security-critical scenarios detecting potential suspicious targets and track them in real-time with minimal response time.
                    </p> */}
                    <ul className="list-disc pl-6">
                    <li>Thesis title: Vision-based tracking and following of a moving target using an unmanned aerial vehicle</li>
                    <li>Developed and validated a closed-loop tracking and following system on a quadrotor, combining Kalman-filter based state estimation with a visual servoing controller, through real-world flight experiments on physical hardware.</li>
                </ul>
                </Experience>
                
                {/* <Experience post={"ROSCon Global"} dateStart={"October 2024"} dateEnd={" "} company={"Open Robotics, Denmark"}>
                    <p>
                    I secured a diversity scholarship to attend the ROSCon Global in Denmark where I had the opportunity to network with companies and ROS contributors. I specifically got extensive hands-on experience by attending the workshops named “Open source, open hardware hand-held mobile mapping system for large scale surveys” which gave exposure to essential processes such as LIDAR odometry and multi-session refinement for large-scale mapping and “ros2_control” where we learned about controller chaining, fallback controllers, and async controllers. 
                    </p>
                </Experience> */}
                
                <Experience post={"Robotics Intern"} dateStart={"July 2024"} dateEnd={"August 2024"} company={"Paltech Robotics GmbH"}>
                    {/* Worked on testing and comparing two new ultrasonic sensors i.e. Bosch and Valeo for the collision avoidance task to include the safety
braking feature (setting thresholds to slow down or stop the robot with ROS2) which involved performing multiple field tests of
different high grass. */}
<ul className="list-disc pl-6"> <li>Evaluated Bosch and Valeo ultrasonic sensors and implemented ROS2-based safety braking logic for collision avoidance on an agricultural field robot.</li>
    <li>Performed extensive outdoor testing in high-grass environments to validate sensor reliability and collision-avoidance performance.</li>
</ul>
                </Experience>

                {/* <Experience post={"B.E. in Software Engineering"} dateStart={2018} dateEnd={2022} company={"Mehran University of Engineering and Technology"} >
                    <p>Agent Based Intelligent Systems, Data Science & Analytics, Simulation & Modeling, Cloud Computing, Statistics and Probablity</p>
                    <p>CGPA 3.96 / 4.00 – Silver Medal Distinction & First Position</p>
                </Experience> */}

                {/* <Experience post={"Développeur Backend"} dateStart={2019} dateEnd={2020} company={"Lyreco Group"} >
                    <p>Développeur en alternance sur un projet Golden Copie.</p>
                    <p>Projet en Java Spring, base MongoDB, sauvegarde des données via une BDD Oracle. Gestion de l’API Rest avec Swagger, routing avec Apache Camel, tests unitaires avec Junit.</p>
                    <p>Travail en TDD, Peer Programing , clean code. Projet dirigé en méthodes agiles avec présence de développeurs Indiens en offshore dans l’équipe. Intégration continue
 avec Jenkins, outils collaboratifs Jira, Bitbucket, Confluence.</p>
                </Experience> */}
            </div>
        </>
        </section>
    )
}

export default Experiences