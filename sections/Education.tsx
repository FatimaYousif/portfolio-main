import SectionBadge from "@/components/SectionBadge"
import Education from "@/components/Education"
import RevealContent from "@/components/ReavealContent"

function Educations() {
    return (
        
         <section id="education" className="scroll-mt-24">
        <>
            <RevealContent>
                <>
                    <SectionBadge sectionName={""} />
                    <h2 className="text-3xl font-extrabold text-center mb-6 md:mb-10">Education</h2>
                </>
            </RevealContent>
            <div className="flex flex-col gap-12">
                <Education post={"Erasmus Mundus Masters in Intelligent Field Robotic Systems"} dateStart={"September 2023"} dateEnd={"July 2025"} company={"Universitat de Girona, Spain & University of Zagreb, Croatia"} >
                    {/* <p>Semester I & II in Girona: Autonomous Systems, Hands-on Planning, Perception, Localization, Manipulation, Machine Learning, Multiview Geometry, Probabilistic Robotics
                    </p>
                    <p>Semester III in Zagreb: Multi-Robot Systems,  Aerial Robotics, Robotic Sensing, Perception, & Actuation, Human-Robot Interaction,
                Deep Learning</p> */}
                <p><strong>Courses:</strong> Hands-on Perception, Hands-on Planning, Autonomous Systems, Multiview Geometry, Machine Learning, Deep Learning, Probabilistic Robotics, Aerial Robotics, Multi-Robots Systems, Hands-on Localization, Robot Sensing, Perception and Actuation, Hands-on Robotic Manipulation, Seminar 1, Human-Robot Interaction</p> 
                <p><strong>Thesis Title:</strong>Vision-Based Tracking and Following of a Moving Target using a Drone.    <br />
                <strong>Thesis grade:</strong> 8.70 / 10</p>
                <p><strong>Overall grade:</strong> 8.45 / 10 (Spanish scale)</p>
                
                
                </Education>
                
                <Education post={"Bachelor of Engineering in Software Engineering"} dateStart={"October 2018"} dateEnd={"November 2022"} company={"Mehran University of Engineering and Technology, Pakistan"} >
                    <p><strong>Relevant Courses: </strong>Data Structures & Algorithms, Discrete Structures, Operations Research, Agent-Based Intelligent Systems, Statistics and Probablity</p>
                    
                    <p><strong>GPA:</strong> 3.96 / 4.00 - Silver Medal Distinction (First Position) </p>
                    
                
                </Education>
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

export default Educations