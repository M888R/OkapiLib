pipeline {
    agent any

    stages {
        stage('Build') {
          steps{
            make [-f Makefile]
            }
        }

    }
}
