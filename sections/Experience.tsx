import SectionBadge from "@/components/SectionBadge"
import Experience from "@/components/Experience"
import RevealContent from "@/components/ReavealContent"

function Experiences() {
    return (
        <>
            <RevealContent>
                <>
                    <SectionBadge sectionName={""} />
                    <h2 className="text-3xl font-extrabold text-center mb-6 md:mb-10">Experiences</h2>
                </>
            </RevealContent>
            <div className="flex flex-col gap-12">
                <Experience post={"ROSCon 2024 Diversity Scholar"} dateStart={"21-10-2024"} dateEnd={"23-10-2024"} company={"Open Robotics, Denmark"}>
                    <p>
                    I secured a diversity scholarship to attend the ROSCon 2024 in Denmark where I had the opportunity to network with companies and ROS contributors globally. I specifically got extensive hands-on experience by attending the workshops named “Open source, open hardware hand-held mobile mapping system for large scale surveys” which gave exposure to essential processes such as LIDAR odometry and multi-session refinement for large-scale mapping and “ros2_control” where we learned about controller chaining, fallback controllers, and async controllers. 
                    </p>
                </Experience>
                
                <Experience post={"Robotics Intern"} dateStart={"06-2024"} dateEnd={"08-2024"} company={"Paltech Robotics GmbH"}>
                    <p>
                    Worked on testing and comparing two new ultrasonic sensors i.e. Bosch and Valeo for the obstacle avoidance task to include the safety
braking feature (setting thresholds to slow down or stop the robot with ROS2) which involved performing multiple field tests of
different high grass.</p>
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
    )
}

export default Experiences