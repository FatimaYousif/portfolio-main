"use client";

import { motion } from "framer-motion"
import Button from "../components/Button"

function Hero() {
    return (
        <div className="flex flex-col items-center justify-center h-full">
            <motion.div
                initial={{ opacity: 0, y: 10 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ delay: 0.5, duration: 1 }}
                className="text-center"
            >
                <p className="text-xl font-bold">Hello, I am </p>
                <h1>Fatima Yousif</h1>
                <p className="text-xl font-bold">MSc Student in Intelligent Field Robotic Systems (Currently, Croatia) </p>
            </motion.div>
            <motion.div
                initial={{ opacity: 0, y: 10 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ delay: 0.8, duration: 1 }}
                className="flex flex-col sm:flex-row mt-12 items-center gap-9"
            >
                <a href="mailto:fyousif30@gmail.com">
                    <Button
                        color={"primary"}
                        text={"Get in touch"}
                    />
                </a>
                <a href="https://drive.google.com/file/d/1WgVvPza-TQUsv5WJqlS0UOZBefIq5JDP/view?usp=drivesdk" target="_blank">
                    <Button
                        color={"secondary"}
                        text={"Download CV"}
                        className={'grow'}
                    />
                </a>
            </motion.div>
        </div>
    )
}

export default Hero