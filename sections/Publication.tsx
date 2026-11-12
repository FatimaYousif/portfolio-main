import SectionBadge from "@/components/SectionBadge"
import Experience from "@/components/Experience"
import RevealContent from "@/components/ReavealContent"

function Publications() {
    return (
        <section id="publications" className="scroll-mt-24">
            <RevealContent>
                <>
                    <SectionBadge sectionName={""} />
                    <h2 className="text-3xl font-extrabold text-center mb-6 md:mb-10">
                        Publications
                    </h2>
                </>
            </RevealContent>

            <div className="flex flex-col gap-12">
                <Experience
                    post={"Security Drone: An End-to-End Framework for Adaptive UAV Patrolling"}
                    dateStart={"2026"}
                    dateEnd={""}
                    company={"IEEE International Conference on Safety, Security, and Rescue Robotics (SSRR)"}
                >
                    <p>
                        Buschman, C. F., van Manen, B. R., <strong> Rustamani, F. Y.</strong>,
                        Rajendra, C. K., & Mersha, A. Y.
                    </p>
                </Experience>
            </div>
        </section>
    )
}

export default Publications