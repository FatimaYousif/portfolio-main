import Icons from "../components/Icons"

function Socials() {
  return (
    <div className="flex gap-12 mb-[50px] items-center">
      <a href="https://github.com/FatimaYousif" target="_blank">
        <Icons name={"github"} size={30}/>
      </a>
      <a href="https://www.linkedin.com/in/fatima-yousif/" target="_blank">
        <Icons name={"linkedin"} size={30}/>
      </a>
      <a href="mailto:fyousif30@gmail.com" target="_blank">
        <Icons name={"mail"} size={25}/>
      </a>
    </div>
  )
}

export default Socials