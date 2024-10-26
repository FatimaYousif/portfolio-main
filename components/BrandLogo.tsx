import Image from "next/image"
import lightLogo from '/public/images/logo-clear-theme.svg';
import darkLogo from '/public/images/logo-dark-theme.svg';

function BrandLogo() {
    return (
        <>
            <Image src={lightLogo} alt="logo Fatima Yousif" className='max-h-12 dark:hidden w-fit' />
            <Image src={darkLogo} alt="logo Fatima Yousif" className='hidden dark:block max-h-12 w-fit' />
        </>
    )
}

export default BrandLogo