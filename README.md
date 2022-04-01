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
`rosver`: which ros version the page is for, right now the possible options are "kinetic" and "noetic"; the page should also be in the correct folder for its version but it's not strictly required
`nav_order`: in which place should this page appear in the navbar group
`nav_exclude`: if set to true the page will be excluded from the navbar

Navbar groups are hardcoded in https://github.com/MoffKalast/learn/blob/84dd6acac33f9ea9bb8d7f94b40e21a6feabd12d/_includes/nav.html#L46 for simplicity and to keep the order correct.

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