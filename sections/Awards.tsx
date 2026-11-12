import SectionBadge from "@/components/SectionBadge"
import Experience from "@/components/Experience"
import RevealContent from "@/components/ReavealContent"

function Awards() {
    return (
         <section id="conferences" className="scroll-mt-24">

        <>
            <RevealContent>
                <>
                    <SectionBadge sectionName={""} />
                    <h2 className="text-3xl font-extrabold text-center mb-6 md:mb-10">
                        Conferences
                    </h2>
                </>
            </RevealContent>

            <div className="flex flex-col gap-12">


                <Experience
                    post={"Open Source Summit + Embedded Linux Conference"}
                    dateStart={"October 2026"}
                    dateEnd={""}
                    company={"The Linux Foundation"}
                >
                       <p> Primarily attended the in PX4 Developer Summit sessions covering autonomous UAV systems, flight control architectures, navigation, multi-UAV systems, PX4-ROS 2 integration, MAVLink, and embedded AI, while engaging with researchers, developers, and industry professionals in the open-source robotics community.</p>
                
                </Experience>


                <Experience
                    post={"ROSCon Global"}
                    dateStart={"October 2024"}
                    dateEnd={""}
                    company={"Open Robotics"}
                >
                    <p>
                        Awarded a diversity scholarship to attend ROSCon Global
                        2024 in Denmark, where I participated in technical
                        workshops on mobile mapping, LiDAR odometry,
                        and ros2_control.
                    </p>
                </Experience>

             


            </div>
        </>
        </section>
    )
}

export default Awards