import About from "@/sections/About";
import Contact from "@/sections/Contact";
import Education from "@/sections/Education";
import Experiences from "@/sections/Experience";
import Hero from "@/sections/Hero";
import MarqueeSection from "@/sections/MarqueeSection";
import Projects from "@/sections/Projects";
import Photos from "@/sections/Photos";

export default function Home() {
  return (
    <>
      <main>
        <section className="h-[70vh] mb-0">
          <Hero />
        </section>
        {/* <section>
          <MarqueeSection />
        </section> */}
        <section>
          <About />
        </section>

        <section>
          <Education/>
        </section>
        

        <section>
          <Experiences />
        </section>
        <section>
          <Projects />
        </section>
        <section>
          <Photos />
        </section>
        <section>
          <Contact />
        </section>
      </main>
    </>
  );
}