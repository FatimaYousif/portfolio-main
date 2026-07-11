import SectionBadge from "@/components/SectionBadge"
import Education from "@/components/Education"
import RevealContent from "@/components/ReavealContent"

function Educations() {
    return (
        <>
            <RevealContent>
                <>
                    <SectionBadge sectionName={""} />
                    <h2 className="text-3xl font-extrabold text-center mb-6 md:mb-10">Education</h2>
                </>
            </RevealContent>
            <div className="flex flex-col gap-12">
                <Education post={"Erasmus Mundus Joint Masters in Intelligent Field Robotic Systems"} dateStart={2023} dateEnd={2025} company={"University of Girona & University of Zagreb"} >
                    <p>Semester I & II in Girona: Autonomous Systems, Hands-on Perception, Planning, Localization (SLAM), Manipulation, Machine Learning, Multiview Geometry, Probabilistic Robotics (Kalman Filtering)
                    </p>
                    <p>Semester III in Zagreb: Aerial Robotics, Multi-Robot Systems, Robotic Sensing, Perception, & Actuation, Human-Robot Interaction,
                Deep Learning, and Ethics & Technology.</p>
                </Education>
                
                <Education post={"Bachelor of Engineering in Software Engineering"} dateStart={2018} dateEnd={2022} company={"Mehran University of Engineering and Technology"} >
                    <p>Agent Based Intelligent Systems, Data Science & Analytics, Simulation & Modeling, Cloud Computing, Statistics and Probablity</p>
                    <p>CGPA 3.96 / 4.00 - Silver Medal Distinction & First Position</p>
                    
                
                </Education>
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

export default Educations