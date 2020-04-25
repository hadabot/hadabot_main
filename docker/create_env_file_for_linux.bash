osstr=`uname -s`

if [[ "$osstr" == "Linux" ]]; then
    echo "Creating a .env file for docker-compose so the containers created can have uid's that match your host linux system, avoiding file permission problems."
    echo "USER_ID=$(id -u)" > .env
    echo "GROUP_ID=$(id -g)" >> .env
else
    echo "Not a linux system: don't need to create an env file for docker-compose"
fi
