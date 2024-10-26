import SectionBadge from "@/components/SectionBadge"
import Icons from "@/components/Icons"
import RevealContent from "@/components/ReavealContent"
import Button from "../components/Button"


function Contact() {
    return (
        <RevealContent style="flex flex-col items-center">
            <>
                <SectionBadge sectionName="" />
                <p className="text-2xl text-center md:text-5xl font-extrabold font-gilroy">Contact Me</p>
                <div className="flex flex-col lg:flex-row w-full max-w-4xl mt-8 gap-12">
                    {/* <div className="flex flex-col items-center lg:items-start lg:w-1/2">
                        <a href="mailto:fyousif30@gmail.com" target="_blank" className="mb-6 mt-10 font-bold text-lg sm:text-xl flex gap-4 items-center">
                            <Icons name="mail" size={20} /> fyousif30@gmail.com
                        </a>
                        <div className="flex flex-col gap-7 mt-6">
                            <a href="https://github.com/FatimaYousif" target="_blank" >
                                <Icons name={"github"} size={30} />
                            </a>
                            <a href="https://www.linkedin.com/in/fatima-yousif/" target="_blank" >
                                <Icons name={"linkedin"} size={30} />
                            </a>
                        </div>
                    </div> */}
                    <div className="flex flex-col items-center lg:items-start lg:w-1/2">
                        <a href="mailto:fyousif30@gmail.com" target="_blank" className="mb-6 mt-10 font-bold text-lg sm:text-xl flex gap-4 items-center">
                            <Icons name="mail" size={20} /> fyousif30@gmail.com
                        </a>
                        <div className="flex flex-row gap-7 mt-6">
                            <a href="https://github.com/FatimaYousif" target="_blank" className="flex justify-center lg:justify-start">
                                <Icons name={"github"} size={30} />
                            </a>
                            <a href="https://www.linkedin.com/in/fatima-yousif/" target="_blank" className="flex justify-center lg:justify-start">
                                <Icons name={"linkedin"} size={30} />
                            </a>
                        </div>
                    </div>
                    <div className="lg:w-1/2">
                        <form action="https://formspree.io/f/xdojynkq" method="POST" className="flex flex-col gap-4 w-full">
                            <div className="flex flex-col">
                                {/* <label htmlFor="name" className="text-lg font-bold">Name</label> */}
                                <input type="text" name="name" placeholder="Name" className="box p-2 border border-gray-300 rounded dark:bg-[#041121] dark:text-white" required />
                            </div>
                            <div className="flex flex-col">
                                {/* <label htmlFor="email" className="text-lg font-bold">Email</label> */}
                                <input type="email" name="email" placeholder="Email" className="box p-2 border border-gray-300 rounded dark:bg-[#041121] dark:text-white" required />
                            </div>
                            <div className="flex flex-col">
                                {/* <label htmlFor="message" className="text-lg font-bold">Message</label> */}
                                <textarea name="message" cols={30} rows={10} className="box p-2 border border-gray-300 rounded dark:bg-[#041121] dark:text-white" placeholder="Message" required></textarea>
                            </div>
                            {/* <button type="submit" className="btn p-2 bg-blue-500 text-white rounded hover:bg-blue-600 transition duration-200">
                                Send <i className="fas fa-paper-plane"></i>
                            </button> */}
                            <Button color="primary" text="Send" className="required" />
                      
                        </form>
                    </div>
                </div>
            </>
        </RevealContent>
    )
}

export default Contact