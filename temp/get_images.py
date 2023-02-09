# download list of images from the web 

chess_images = [
    'https://images.unsplash.com/photo-1625750331870-624de6fd3452?ixlib=rb-4.0.3&ixid=MnwxMjA3fDB8MHxzZWFyY2h8MTh8fGNoZXNzJTIwYm9hcmR8ZW58MHx8MHx8&auto=format&fit=crop&w=500&q=60',
    'https://images.unsplash.com/photo-1528819622765-d6bcf132f793?ixlib=rb-4.0.3&ixid=MnwxMjA3fDB8MHxzZWFyY2h8MTB8fGNoZXNzJTIwYm9hcmR8ZW58MHx8MHx8&auto=format&fit=crop&w=500&q=60',
    'https://images.unsplash.com/photo-1611195974226-a6a9be9dd763?ixlib=rb-4.0.3&ixid=MnwxMjA3fDB8MHxzZWFyY2h8Mnx8Y2hlc3MlMjBib2FyZHxlbnwwfHwwfHw%3D&auto=format&fit=crop&w=500&q=60',
    'https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcQ-tyiF-Mpa-HoAPnc9gjzpSfl5drlddiVjUQ&usqp=CAU',
    'https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcTdEv2dGrVnvW_4dX2RpV2f8QqSjxyTBwJgLg&usqp=CAU',
    'https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcRI0Ha6-Thw0_ODTB_aWerZJTLQ-5A3mNVLPw&usqp=CAU',
    'https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcRJptcY1IA4yMemUeY_5n0ZAJCl5pE2Spz9gw&usqp=CAU',
]

import subprocess

def runcmd(cmd, verbose = False, *args, **kwargs):

    process = subprocess.Popen(
        cmd,
        stdout = subprocess.PIPE,
        stderr = subprocess.PIPE,
        text = True,
        shell = True
    )
    std_out, std_err = process.communicate()
    if verbose:
        print(std_out.strip(), std_err)
    pass

for i,url in enumerate(chess_images):
  name = 'chess'+str(i)+'.jpg'
  runcmd(f"wget --output-document={name} {url}")
  chess_images[i] = name
  