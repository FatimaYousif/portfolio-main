"use client";

import RevealContent from "@/components/ReavealContent"
import SectionBadge from "@/components/SectionBadge"
import { motion } from "framer-motion"
import Image from "next/image"

function About() {
    return (
        <motion.div
            initial={{ opacity: 0, y: 10 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ delay: 1.2, duration: 1 }}
        >
            <RevealContent>
                <>
                    <SectionBadge sectionName="" />
                    <div className="flex flex-col md:flex-row md:gap-6">
                        <div className="w-12/12 md:w-5/12 mb-6 flex justify-center md:self-start">
                            <Image src={"/images/Profile.JPG"} className="self-center dark:hidden" alt="" height={400} width={400} />
                            <Image src={"/images/Profile.JPG"} className="self-center hidden dark:block" alt="" height={400} width={400} />
                        </div>
                        <div className="w-12/12 md:w-7/12 xl:w-5/12">
                            <h2 className="text-3xl font-bold mb-6">About Me</h2>
                            <div className="flex flex-col gap-4" style={{ textAlign: 'justify' }}>
                                <p>
                                I am an Erasmus Mundus Joint Master’s Scholar in Intelligent Field Robotic Systems at Universitat de Girona, Spain, and the University of Zagreb, Croatia. 
                                My research interests span a diverse range of cutting-edge disciplines, including robotics, computer vision, and deep/machine learning. 
                                </p>
                                <p>
                                I am passionate about exploring the intersection of these fields to push the boundaries of technology and create innovative solutions that address complex real-world challenges.
                                
                                </p>
                                <p>
                                Additionally, I have proven leadership qualities demonstrated in global communities including Google Developer Student Clubs, TEDx, Google Developer Groups Live, U.S. Embassy programs, and 10Pearls.
                                </p>
                            </div>
                        </div>
                    </div>
                </>
            </RevealContent>
        </motion.div>
    )
}

export default About