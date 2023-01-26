# Learn Docs

Documentation for all Ubiquity software and robots.

Each page needs to have the following defined at the top:

    ---
    title: "Magni Power Controls"
    permalink: noetic_overview_magni_key
    group: overview
    rosver: noetic
    nav_order: 3
    nav_exclude: false
    ---


`title`: what the nav bar will show the page as

`permalink`: unique identifier for the page, best set as `<ros version>_<navbar group>_something` and **MUST BE UNIQUE** otherwise only one of the same-named pages will be accesible

`group`: the navbar group the page should appear in, see below for current options

`rosver`: which ros version the page is for, right now the possible options are "kinetic" and "noetic"; the page should also be in the correct folder for its version but it's not strictly required, the nav menu will filter pages based on this parameter only

`nav_order`: in which place should this page appear in the navbar group

`nav_exclude`: if set to true the page will be excluded from the navbar; note: hidden pages do not need a `rosver` if it's not relevant for the page, since the nav won't be filtering them anyway

Navbar groups are hardcoded in https://github.com/UbiquityRobotics/learn/blob/b9445842f965e29c937405db88ae9717b9152789/_includes/nav.html#L95 for simplicity and to keep the order correct.

## Test Locally

Github can be very slow (up to 20 mins) to show changes so testing stuff locally can be a good option.

    # Install jekyll, and Ruby version 2.7
    https://jekyllrb.com/docs/installation/

    # Install bundler
    gem install bundler
    
    # Ruby
    rvm 2.7

    cd learn
    bundle install
    bundle exec jekyll serve

Then you can see the site on [http://localhost:4000](http://localhost:4000).

## Docker
To test changes locally, you can also use docker.

If you have docker installed, you can run the following command in the learn directory.

    sudo docker run -it --rm -v "$PWD":/usr/src/app -p "4000:4000" starefossen/github-pages
