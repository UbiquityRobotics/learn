# learn
Tutorials for Ubiquity's robots


## Docker
To test changes locally, you can use docker.

If you have docker installed, you can run the following command in the learn directory.

    sudo docker run -it --rm -v "$PWD":/usr/src/app -p "4000:4000" starefossen/github-pages

Then you can see the site on [http://localhost:4000](http://localhost:4000).

