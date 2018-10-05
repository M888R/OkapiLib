pipeline {
    agent any

    stages {
        stage('Build') {
            steps {
                sh('Makefile')
            }
        }
        stage('Test') {
            steps {
                echo 'Testing..'
            }
        }
        stage('Deploy') {
            steps {
                echo 'Deploying....'
            }
        }
    }
}
