
import RevealContent from "@/components/ReavealContent"
import SectionBadge from "@/components/SectionBadge"


export default function Photos() {
  return (
     <>
      <RevealContent>
        <>
          <SectionBadge sectionName={""} />
          <h2 className="text-3xl font-extrabold text-center mb-6 md:mb-10">Highlights</h2>
        </>
      </RevealContent>

    {/* <section id="field-tests" className="py-20"> */}
      {/* <h2 className="text-3xl md:text-5xl font-extrabold font-gilroy text-center mb-8">
        Highlights
      </h2> */}
      
        {/* <h2 className="text-3xl font-extrabold text-center mb-6 md:mb-10">Highlights</h2> */}

      <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
        <img src="images/gallery/drone2_2.jpg" alt="Drone Test 2" />

        <img src="images/gallery/nestfly_2.JPG" alt="Field Test 1" />
        <img src="images/gallery/ctt3.jpg" alt="Field Test 3" />

        {/* <img src="images/gallery/robor2.jpg" alt="Field Test 2" /> */}

        <img src="images/gallery/robor.jpg" alt="Robotics Event" />
        
        
        <img src="images/gallery/roscon.jpeg" alt="ROS Conference" />
        <img src="images/gallery/paltech.jpeg" alt="Paltech Event" />

        <img src="images/gallery/nestfly3.JPG" alt="Robotics Event" />
        
        <img src="images/gallery/ut.png" alt="University of Twente" />


        <img src="images/gallery/girlsday.jpg" alt="Girls Day Event" />
        
        
        {/* <img src="images/gallery/nwo2.jpg" alt="NWO Event 1" /> */}
        <img src="images/gallery/nwo.jpeg" alt="NWO Event 2" />
        {/* <img src="images/gallery/drone4.jpeg" alt="Drone Test" /> */}


        {/* <img src="images/gallery/nestfly.jpeg" alt="NestFly Event" /> */}
      </div>
    {/* </section> */}
    </>
  );
}





// export default function Photos() {
//   return (
//     <section id="field-tests" className="py-20">
//       <p className="text-2xl text-center md:text-5xl font-extrabold font-gilroy">Gallery</p>
//       <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
//         <img src="images/gallery/ctt1.jpg" alt="Field Test 1" />
//         <img src="images/gallery/ctt2.jpeg" alt="Field Test 2" />
//         <img src="images/gallery/ctt3.jpg" alt="Field Test 3" />
//         <img src="images/gallery/roscon.jpeg" alt="Field Test 3" />
//         <img src="images/gallery/paltech.jpeg" alt="Field Test 3" />

//         <img src="images/gallery/ut.png" alt="Field Test 3" />
//         <img src="images/gallery/drone.png" alt="Field Test 3" />

//         <img src="images/gallery/drone2.jpeg" alt="Field Test 3" />
        
        

//         <img src="images/gallery/girlsday.jpg" alt="Field Test 3" />
//         <img src="images/gallery/robor.jpg" alt="Field Test 3" />
//         <img src="images/gallery/nwo2.jpg" alt="Field Test 3" />
//         <img src="images/gallery/nwo.jpeg" alt="Field Test 3" />

//         {/* <img src="images/gallery/nestfly.jpeg" alt="Field Test 3" /> */}
        
        
        
//       </div>
//     </section>
//   );
// }