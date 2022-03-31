# Learn Docs

Documentation for all Ubiquity software and robots.

## Test Locally

Github can be very slow (up to 20 mins) to show changes so testing stuff locally can be a good option.

    # Install jekyll
    https://jekyllrb.com/docs/installation/

    # Install bundler
    gem install bundler

    cd learn
    bundle install
    bundle exec jekyll serve

Then you can see the site on [http://localhost:4000](http://localhost:4000).

## Docker
To test changes locally, you can also use docker.

If you have docker installed, you can run the following command in the learn directory.

    sudo docker run -it --rm -v "$PWD":/usr/src/app -p "4000:4000" starefossen/github-pages